#!/usr/bin/env python3
"""Record ball-follow inputs, controller outputs and actuator state to CSV."""

from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from usv_interfaces.msg import (
    BallDetection,
    BallTargetState,
    PropulsionCommand,
    PwmOutputState,
)


class TelemetryRecorder(Node):
    """Sample the latest messages onto one time-aligned CSV row."""

    def __init__(self) -> None:
        super().__init__("ball_follow_telemetry_recorder")
        self.declare_parameter("output_directory", str(Path.home() / "Downloads" / "usv_logs"))
        self.declare_parameter("record_rate", 20.0)

        output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        record_rate = float(self.get_parameter("record_rate").value)
        if not math.isfinite(record_rate) or record_rate <= 0.0:
            raise ValueError("record_rate must be positive")
        output_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = output_directory / f"ball_follow_{stamp}.csv"
        self._file = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self._field_names())
        self._writer.writeheader()

        self._lock = threading.Lock()
        self._detection: BallDetection | None = None
        self._target: BallTargetState | None = None
        self._imu: Imu | None = None
        self._desired_yaw_rate: Float32 | None = None
        self._automatic_command: PropulsionCommand | None = None
        self._final_command: PropulsionCommand | None = None
        self._pwm: PwmOutputState | None = None
        self._automatic_mode: Bool | None = None
        self._follow_active: Bool | None = None

        self.create_subscription(BallDetection, "/ball/detection", self._set_detection, 10)
        self.create_subscription(BallTargetState, "/ball/target_state", self._set_target, 10)
        self.create_subscription(Imu, "/imu/data_raw", self._set_imu, qos_profile_sensor_data)
        self.create_subscription(Float32, "/ball/desired_yaw_rate", self._set_desired_rate, 10)
        self.create_subscription(PropulsionCommand, "/propulsion/automatic_command", self._set_auto_command, 10)
        self.create_subscription(PropulsionCommand, "/propulsion/command", self._set_final_command, 10)
        self.create_subscription(PwmOutputState, "/actuators/pwm_state", self._set_pwm, 10)
        self.create_subscription(Bool, "/control/automatic_mode", self._set_auto_mode, 10)
        self.create_subscription(Bool, "/ball/follow_active", self._set_follow_active, 10)
        self.create_timer(1.0 / record_rate, self._record)
        self.get_logger().info(
            f"Recording {record_rate:.1f} Hz telemetry to {self._path}"
        )

    @staticmethod
    def _field_names() -> list[str]:
        return [
            "time_iso", "ros_time_sec", "automatic_mode", "follow_active",
            "detected", "detection_confidence", "detection_center_x",
            "detection_area_ratio", "inference_ms", "target_valid",
            "tracking_state", "bearing_rad", "bearing_deg",
            "bearing_rate_rad_s", "detection_age_sec", "target_imu_valid",
            "target_yaw_rate_rad_s", "imu_ax_m_s2", "imu_ay_m_s2",
            "imu_az_m_s2", "imu_gx_rad_s", "imu_gy_rad_s", "imu_gz_rad_s",
            "imu_qx", "imu_qy", "imu_qz", "imu_qw", "desired_yaw_rate_rad_s",
            "automatic_left_percent", "automatic_right_percent",
            "final_left_percent", "final_right_percent", "final_source",
            "pwm_requested_left_percent", "pwm_requested_right_percent",
            "pwm_applied_left_percent", "pwm_applied_right_percent",
            "pwm_left_us", "pwm_right_us", "watchdog_active",
            "emergency_stop_active", "dry_run",
        ]

    def _assign(self, name: str, message) -> None:
        with self._lock:
            setattr(self, name, message)

    def _set_detection(self, msg): self._assign("_detection", msg)
    def _set_target(self, msg): self._assign("_target", msg)
    def _set_imu(self, msg): self._assign("_imu", msg)
    def _set_desired_rate(self, msg): self._assign("_desired_yaw_rate", msg)
    def _set_auto_command(self, msg): self._assign("_automatic_command", msg)
    def _set_final_command(self, msg): self._assign("_final_command", msg)
    def _set_pwm(self, msg): self._assign("_pwm", msg)
    def _set_auto_mode(self, msg): self._assign("_automatic_mode", msg)
    def _set_follow_active(self, msg): self._assign("_follow_active", msg)

    def _record(self) -> None:
        with self._lock:
            detection = self._detection
            target = self._target
            imu = self._imu
            desired = self._desired_yaw_rate
            automatic = self._automatic_command
            final = self._final_command
            pwm = self._pwm
            automatic_mode = self._automatic_mode
            follow_active = self._follow_active
        now = self.get_clock().now()
        row = {name: "" for name in self._field_names()}
        row.update(time_iso=datetime.now().isoformat(timespec="milliseconds"),
                   ros_time_sec=f"{now.nanoseconds / 1.0e9:.6f}")
        if automatic_mode is not None: row["automatic_mode"] = int(automatic_mode.data)
        if follow_active is not None: row["follow_active"] = int(follow_active.data)
        if detection is not None:
            row.update(detected=int(detection.detected), detection_confidence=detection.confidence,
                       detection_center_x=detection.center_x,
                       detection_area_ratio=detection.area_ratio,
                       inference_ms=detection.inference_ms)
        if target is not None:
            row.update(target_valid=int(target.target_valid), tracking_state=target.tracking_state,
                       bearing_rad=target.bearing_rad,
                       bearing_deg=math.degrees(target.bearing_rad),
                       bearing_rate_rad_s=target.bearing_rate_rad_s,
                       detection_age_sec=target.detection_age_sec,
                       target_imu_valid=int(target.imu_valid),
                       target_yaw_rate_rad_s=target.yaw_rate_rad_s)
        if imu is not None:
            row.update(imu_ax_m_s2=imu.linear_acceleration.x,
                       imu_ay_m_s2=imu.linear_acceleration.y,
                       imu_az_m_s2=imu.linear_acceleration.z,
                       imu_gx_rad_s=imu.angular_velocity.x,
                       imu_gy_rad_s=imu.angular_velocity.y,
                       imu_gz_rad_s=imu.angular_velocity.z,
                       imu_qx=imu.orientation.x, imu_qy=imu.orientation.y,
                       imu_qz=imu.orientation.z, imu_qw=imu.orientation.w)
        if desired is not None: row["desired_yaw_rate_rad_s"] = desired.data
        if automatic is not None:
            row.update(automatic_left_percent=automatic.channel_1_percent,
                       automatic_right_percent=automatic.channel_2_percent)
        if final is not None:
            row.update(final_left_percent=final.channel_1_percent,
                       final_right_percent=final.channel_2_percent,
                       final_source=final.source)
        if pwm is not None:
            row.update(pwm_requested_left_percent=pwm.requested_percent[0],
                       pwm_requested_right_percent=pwm.requested_percent[1],
                       pwm_applied_left_percent=pwm.applied_percent[0],
                       pwm_applied_right_percent=pwm.applied_percent[1],
                       pwm_left_us=pwm.pulse_width_us[0], pwm_right_us=pwm.pulse_width_us[1],
                       watchdog_active=int(pwm.watchdog_active),
                       emergency_stop_active=int(pwm.emergency_stop_active),
                       dry_run=int(pwm.dry_run))
        self._writer.writerow(row)
        self._file.flush()

    def destroy_node(self):
        if not self._file.closed:
            self._file.flush()
            self._file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryRecorder()
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
