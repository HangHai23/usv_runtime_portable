#!/usr/bin/env python3
"""Cascaded bearing/yaw-rate controller publishing automatic propulsion."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from usv_interfaces.msg import BallTargetState, PropulsionCommand

from .control_math import (
    clamp,
    alignment_active,
    desired_yaw_rate,
    mix_same_direction_or_pivot,
    scheduled_throttle,
    slew,
)


class BallFollowController(Node):
    def __init__(self) -> None:
        super().__init__("ball_follow_controller")
        self.declare_parameter("target_topic", "/ball/target_state")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/propulsion/automatic_command")
        self.declare_parameter("enable_topic", "")
        self.declare_parameter("bearing_deadband_rad", 0.035)
        self.declare_parameter("bearing_kp", 2.0)
        self.declare_parameter("maximum_yaw_rate_rad_s", 0.70)
        self.declare_parameter("yaw_rate_kp", 28.0)
        self.declare_parameter("yaw_brake_preview_sec", 0.0)
        self.declare_parameter("maximum_steering_percent", 100.0)
        self.declare_parameter("imu_yaw_rate_reversed", False)
        self.declare_parameter("steering_reversed", False)
        self.declare_parameter("cruise_throttle_percent", 15.0)
        self.declare_parameter("steering_slowdown_angle_rad", 0.45)
        self.declare_parameter("stop_area_ratio", 0.20)
        self.declare_parameter("predicting_throttle_scale", 0.30)
        self.declare_parameter("output_slew_rate_percent_s", 60.0)
        self.declare_parameter("target_timeout", 0.20)
        self.declare_parameter("imu_timeout", 0.20)
        self.declare_parameter("output_rate", 20.0)
        self.declare_parameter("alignment_pivot_enabled", False)
        self.declare_parameter("alignment_enter_angle_deg", 15.0)
        self.declare_parameter("alignment_exit_angle_deg", 7.0)
        self.declare_parameter("alignment_pivot_percent", 35.0)
        self._alignment = False
        self._pivot_enabled = bool(self.get_parameter('alignment_pivot_enabled').value)
        self._pivot_enter = math.radians(float(self.get_parameter('alignment_enter_angle_deg').value))
        self._pivot_exit = math.radians(float(self.get_parameter('alignment_exit_angle_deg').value))
        self._pivot_limit = float(self.get_parameter('alignment_pivot_percent').value)
        if not (0 < self._pivot_exit < self._pivot_enter <= math.pi and 0 < self._pivot_limit <= 100):
            raise ValueError('invalid alignment angles/power')
        self.declare_parameter("navigation_model_enabled", False)
        self.declare_parameter("bidirectional_enabled", False)
        self._bidirectional = bool(self.get_parameter("bidirectional_enabled").value)
        self.declare_parameter("cruise_speed_m_s", 2.6892)
        self.declare_parameter("speed_gain_m_s_per_us", 0.00415)
        self.declare_parameter("turn_gain_rad_s_per_us", 0.00335)
        self.declare_parameter("command_span_us", 810.0)
        self.declare_parameter("yaw_feedback_gain", 0.8)
        self._navigation_model = bool(self.get_parameter("navigation_model_enabled").value)
        self._model = {key: float(self.get_parameter(key).value) for key in (
            "cruise_speed_m_s", "speed_gain_m_s_per_us", "turn_gain_rad_s_per_us",
            "command_span_us", "yaw_feedback_gain")}
        if any(not math.isfinite(v) or v <= 0 for v in self._model.values()):
            raise ValueError("navigation model parameters must be finite and positive")

        parameter_names = [
            "bearing_deadband_rad", "bearing_kp", "maximum_yaw_rate_rad_s",
            "yaw_rate_kp", "yaw_brake_preview_sec", "maximum_steering_percent", "cruise_throttle_percent",
            "steering_slowdown_angle_rad", "stop_area_ratio",
            "predicting_throttle_scale", "output_slew_rate_percent_s",
            "target_timeout", "imu_timeout", "output_rate",
        ]
        values = {name: float(self.get_parameter(name).value) for name in parameter_names}
        self._deadband = values["bearing_deadband_rad"]
        self._bearing_kp = values["bearing_kp"]
        self._max_yaw_rate = values["maximum_yaw_rate_rad_s"]
        self._yaw_rate_kp = values["yaw_rate_kp"]
        self._yaw_preview = values["yaw_brake_preview_sec"]
        if not math.isfinite(self._yaw_preview) or self._yaw_preview < 0.0:
            raise ValueError("yaw_brake_preview_sec must be finite and nonnegative")
        self._max_steering = values["maximum_steering_percent"]
        self._cruise = values["cruise_throttle_percent"]
        self._slowdown_angle = values["steering_slowdown_angle_rad"]
        self._stop_area = values["stop_area_ratio"]
        self._predicting_scale = values["predicting_throttle_scale"]
        self._slew_rate = values["output_slew_rate_percent_s"]
        self._target_timeout = values["target_timeout"]
        self._imu_timeout = values["imu_timeout"]
        self._output_rate = values["output_rate"]
        self._imu_reversed = bool(self.get_parameter("imu_yaw_rate_reversed").value)
        self._steering_reversed = bool(self.get_parameter("steering_reversed").value)
        if min(self._max_yaw_rate, self._max_steering, self._slowdown_angle,
               self._stop_area, self._slew_rate, self._target_timeout,
               self._imu_timeout, self._output_rate) <= 0.0:
            raise ValueError("controller limits, timeouts, rates and stop_area must be positive")
        if not 0.0 <= self._predicting_scale <= 1.0:
            raise ValueError("predicting_throttle_scale must be in 0..1")
        if not 0.0 <= self._cruise <= 100.0:
            raise ValueError("cruise_throttle_percent must be in 0..100")
        if not 0.0 < self._max_steering <= 100.0:
            raise ValueError("maximum_steering_percent must be in 0..100")

        self._lock = threading.Lock()
        self._target = None
        self._last_target = 0.0
        self._yaw_rate = 0.0
        self._last_imu = 0.0
        self._outputs = [0.0, 0.0]
        self._enable_topic = str(self.get_parameter("enable_topic").value)
        self._enabled = not bool(self._enable_topic)
        self._last_enable = 0.0
        self._publisher = self.create_publisher(
            PropulsionCommand, str(self.get_parameter("output_topic").value), 10
        )
        self._desired_rate_publisher = self.create_publisher(
            Float32, "/ball/desired_yaw_rate", 10
        )
        self._active_publisher = self.create_publisher(Bool, "/ball/follow_active", 10)
        self.create_subscription(
            BallTargetState,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        if self._enable_topic:
            self.create_subscription(Bool, self._enable_topic, self._on_enable, 10)
        self.create_timer(1.0 / self._output_rate, self._update)
        self.get_logger().info(
            f"Ball-follow cascaded controller ready at {self._output_rate:.1f} Hz; "
            f"cruise={self._cruise:.1f}%, steering limit={self._max_steering:.1f}%"
        )

    def _on_target(self, message: BallTargetState) -> None:
        with self._lock:
            self._target = message
            self._last_target = time.monotonic()

    def _on_enable(self, message: Bool) -> None:
        with self._lock:
            self._enabled = bool(message.data)
            self._last_enable = time.monotonic()

    def _on_imu(self, message: Imu) -> None:
        yaw_rate = float(message.angular_velocity.z)
        if not math.isfinite(yaw_rate):
            return
        if self._imu_reversed:
            yaw_rate = -yaw_rate
        with self._lock:
            self._yaw_rate = yaw_rate
            self._last_imu = time.monotonic()

    def _publish_zero(self) -> None:
        self._alignment = False
        self._outputs = [0.0, 0.0]
        self._publish_command(0.0, 0.0)
        self._desired_rate_publisher.publish(Float32(data=0.0))
        self._active_publisher.publish(Bool(data=False))

    def _publish_command(self, left: float, right: float) -> None:
        message = PropulsionCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.channel_1_percent = float(clamp(left, -100.0, 100.0))
        message.channel_2_percent = float(clamp(right, -100.0, 100.0))
        message.source = "ball_follow"
        self._publisher.publish(message)

    def _update(self) -> None:
        now = time.monotonic()
        with self._lock:
            target = self._target
            target_fresh = target is not None and now - self._last_target <= self._target_timeout
            imu_fresh = self._last_imu > 0.0 and now - self._last_imu <= self._imu_timeout
            yaw_rate = self._yaw_rate
        if (
            (self._enable_topic and (not self._enabled or now-self._last_enable > self._target_timeout))
            or not target_fresh
            or not target.target_valid
            or not imu_fresh
            or not math.isfinite(float(target.bearing_rad))
            or not math.isfinite(float(target.area_ratio))
        ):
            self._publish_zero()
            return

        self._alignment = self._pivot_enabled and self._bidirectional and alignment_active(
            self._alignment, float(target.bearing_rad), self._pivot_enter, self._pivot_exit)
        desired_rate = desired_yaw_rate(
            float(target.bearing_rad) - self._yaw_preview * yaw_rate,
            self._deadband, self._bearing_kp, self._max_yaw_rate
        )
        steering = self._yaw_rate_kp * (desired_rate - yaw_rate)
        steering = clamp(steering, -self._max_steering, self._max_steering)
        if self._steering_reversed:
            steering = -steering
        throttle = scheduled_throttle(
            self._cruise,
            abs(float(target.bearing_rad)),
            self._slowdown_angle,
            float(target.area_ratio),
            self._stop_area,
        )
        if target.tracking_state == BallTargetState.PREDICTING:
            throttle *= self._predicting_scale
        left_target, right_target = mix_same_direction_or_pivot(throttle, steering)
        if self._navigation_model:
            # Right-turn-positive convention: left motor supplies the larger PWM.
            # Use the actual target bearing for slowdown, not the IMU braking
            # bearing: a rapidly turning vessel must not accelerate prematurely.
            speed_factor = clamp(1.0 - abs(float(target.bearing_rad)) / self._slowdown_angle, 0., 1.)
            if target.tracking_state == BallTargetState.PREDICTING:
                speed_factor *= self._predicting_scale
            common = speed_factor * self._model["cruise_speed_m_s"] / self._model["speed_gain_m_s_per_us"]
            difference = (desired_rate + self._model["yaw_feedback_gain"] *
                          (desired_rate - yaw_rate)) / self._model["turn_gain_rad_s_per_us"]
            differential_limit = self._model["command_span_us"] * (2 if self._bidirectional else 1)
            difference = clamp(difference, -differential_limit, differential_limit)
            if self._steering_reversed:
                difference = -difference
            left_us, right_us = max(0., common + difference/2), max(0., common - difference/2)
            # Preserve requested differential with a stopped inner motor when
            # forward common demand is small (forward-only ESCs cannot reverse).
            if self._bidirectional:
                left_us, right_us = common+difference/2, common-difference/2
            elif common < abs(difference)/2:
                left_us, right_us = (abs(difference), 0.) if difference >= 0 else (0., abs(difference))
            lower = -100 if self._bidirectional else 0
            left_target = clamp(left_us / self._model["command_span_us"] * 100, lower, 100)
            right_target = clamp(right_us / self._model["command_span_us"] * 100, lower, 100)
            if float(target.area_ratio) >= self._stop_area and not self._alignment:
                self._publish_zero()
                return
        maximum_delta = self._slew_rate / self._output_rate
        if self._alignment:
            # Remove common propulsion; slew only the signed differential.
            requested = clamp((left_target-right_target)/2, -self._pivot_limit, self._pivot_limit)
            turn = slew((self._outputs[0]-self._outputs[1])/2, requested, maximum_delta)
            turn = clamp(turn, -self._pivot_limit, self._pivot_limit)
            self._outputs = [turn, -turn]
            self._publish_command(turn, -turn)
            self._desired_rate_publisher.publish(Float32(data=float(desired_rate)))
            self._active_publisher.publish(Bool(data=True))
            return
        left = slew(self._outputs[0], left_target, maximum_delta)
        right = slew(self._outputs[1], right_target, maximum_delta)
        self._outputs = [left, right]
        self._publish_command(left, right)
        self._desired_rate_publisher.publish(Float32(data=float(desired_rate)))
        self._active_publisher.publish(Bool(data=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallFollowController()
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
