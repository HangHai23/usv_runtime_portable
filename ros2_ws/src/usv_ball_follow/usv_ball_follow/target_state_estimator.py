#!/usr/bin/env python3
"""Estimate a continuous target bearing from YOLO detections and IMU yaw rate."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from usv_interfaces.msg import BallDetection, BallTargetState

from .control_math import clamp, pixel_center_to_bearing


class TargetStateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("target_state_estimator")
        self.declare_parameter("detection_topic", "/ball/detection")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/ball/target_state")
        self.declare_parameter("horizontal_fov_deg", 70.0)
        self.declare_parameter("camera_horizontal_reversed", False)
        self.declare_parameter("imu_yaw_rate_reversed", False)
        self.declare_parameter("filter_alpha", 0.55)
        self.declare_parameter("filter_beta", 0.08)
        self.declare_parameter("maximum_target_drift_rad_s", 1.0)
        self.declare_parameter("outlier_gate_rad", 0.50)
        self.declare_parameter("minimum_confidence", 0.50)
        self.declare_parameter("prediction_timeout", 0.35)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("imu_timeout", 0.20)
        self.declare_parameter("output_rate", 20.0)

        self._fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self._camera_reversed = bool(
            self.get_parameter("camera_horizontal_reversed").value
        )
        self._imu_reversed = bool(self.get_parameter("imu_yaw_rate_reversed").value)
        self._alpha = float(self.get_parameter("filter_alpha").value)
        self._beta = float(self.get_parameter("filter_beta").value)
        self._max_drift = float(
            self.get_parameter("maximum_target_drift_rad_s").value
        )
        self._outlier_gate = float(self.get_parameter("outlier_gate_rad").value)
        self._minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self._prediction_timeout = float(
            self.get_parameter("prediction_timeout").value
        )
        self._lost_timeout = float(self.get_parameter("lost_timeout").value)
        self._imu_timeout = float(self.get_parameter("imu_timeout").value)
        output_rate = float(self.get_parameter("output_rate").value)
        if not 0.0 < self._fov < math.pi:
            raise ValueError("horizontal_fov_deg must be in 0..180")
        if not 0.0 <= self._alpha <= 1.0 or not 0.0 <= self._beta <= 1.0:
            raise ValueError("filter_alpha and filter_beta must be in 0..1")
        if self._prediction_timeout <= 0.0 or self._lost_timeout <= self._prediction_timeout:
            raise ValueError("lost_timeout must be greater than prediction_timeout")
        if output_rate <= 0.0:
            raise ValueError("output_rate must be positive")

        self._lock = threading.Lock()
        self._bearing = 0.0
        self._drift_rate = 0.0
        self._yaw_rate = 0.0
        self._area_ratio = 0.0
        self._confidence = 0.0
        self._last_detection = 0.0
        self._last_accepted_detection = 0.0
        self._last_imu = 0.0
        self._last_predict = time.monotonic()
        self._initialized = False
        self._publisher = self.create_publisher(
            BallTargetState,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            BallDetection,
            str(self.get_parameter("detection_topic").value),
            self._on_detection,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / output_rate, self._publish)
        self.get_logger().info(
            f"Target estimator ready: FOV={math.degrees(self._fov):.1f} deg, "
            f"output={output_rate:.1f} Hz"
        )

    def _predict_locked(self, now: float) -> None:
        dt = clamp(now - self._last_predict, 0.0, 0.2)
        if self._initialized and dt > 0.0:
            imu_fresh = self._last_imu > 0.0 and now - self._last_imu <= self._imu_timeout
            yaw_rate = self._yaw_rate if imu_fresh else 0.0
            self._bearing += (self._drift_rate - yaw_rate) * dt
            self._bearing = clamp(self._bearing, -self._fov * 0.75, self._fov * 0.75)
        self._last_predict = now

    def _on_imu(self, message: Imu) -> None:
        yaw_rate = float(message.angular_velocity.z)
        if not math.isfinite(yaw_rate):
            return
        if self._imu_reversed:
            yaw_rate = -yaw_rate
        now = time.monotonic()
        with self._lock:
            self._predict_locked(now)
            self._yaw_rate = yaw_rate
            self._last_imu = now

    def _on_detection(self, message: BallDetection) -> None:
        now = time.monotonic()
        if not message.detected or message.confidence < self._minimum_confidence:
            return
        measured = pixel_center_to_bearing(
            message.center_x, self._fov, self._camera_reversed
        )
        with self._lock:
            self._predict_locked(now)
            reacquiring = (
                not self._initialized
                or now - self._last_detection > self._prediction_timeout
            )
            residual = measured - self._bearing
            if not reacquiring and abs(residual) > self._outlier_gate:
                return
            if reacquiring:
                self._bearing = measured
                self._drift_rate = 0.0
            else:
                interval = max(0.02, now - self._last_accepted_detection)
                self._bearing += self._alpha * residual
                self._drift_rate += self._beta * residual / interval
                self._drift_rate = clamp(
                    self._drift_rate, -self._max_drift, self._max_drift
                )
            self._area_ratio = max(0.0, float(message.area_ratio))
            self._confidence = float(message.confidence)
            self._last_detection = now
            self._last_accepted_detection = now
            self._initialized = True

    def _publish(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._predict_locked(now)
            detection_age = (
                now - self._last_detection if self._last_detection > 0.0 else math.inf
            )
            imu_age = now - self._last_imu if self._last_imu > 0.0 else math.inf
            imu_valid = imu_age <= self._imu_timeout
            if not self._initialized or detection_age > self._lost_timeout:
                state = BallTargetState.LOST
                valid = False
            elif detection_age <= self._prediction_timeout:
                state = BallTargetState.TRACKING
                valid = True
            else:
                state = BallTargetState.PREDICTING
                valid = True
            message = BallTargetState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "camera_optical_frame"
            message.target_valid = valid
            message.tracking_state = state
            message.bearing_rad = float(self._bearing)
            message.bearing_rate_rad_s = float(self._drift_rate - (self._yaw_rate if imu_valid else 0.0))
            message.area_ratio = float(self._area_ratio)
            message.confidence = float(self._confidence)
            message.detection_age_sec = float(detection_age if math.isfinite(detection_age) else 1.0e6)
            message.imu_valid = imu_valid
            message.yaw_rate_rad_s = float(self._yaw_rate if imu_valid else 0.0)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
