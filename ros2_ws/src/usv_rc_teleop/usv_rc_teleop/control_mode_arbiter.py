#!/usr/bin/env python3
"""Select manual or automatic propulsion under RC-link supervision."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from usv_interfaces.msg import CrsfChannels, PropulsionCommand


class ControlModeArbiter(Node):
    def __init__(self) -> None:
        super().__init__("control_mode_arbiter")
        self.declare_parameter("mode_channel", 8)
        self.declare_parameter("automatic_above_us", 1500)
        self.declare_parameter("rc_timeout", 0.5)
        self.declare_parameter("manual_command_timeout", 0.2)
        self.declare_parameter("automatic_command_timeout", 0.2)
        self.declare_parameter("output_rate", 50.0)
        self.declare_parameter("force_automatic_without_rc", False)

        self._mode_index = int(self.get_parameter("mode_channel").value) - 1
        self._automatic_above_us = int(
            self.get_parameter("automatic_above_us").value
        )
        self._rc_timeout = float(self.get_parameter("rc_timeout").value)
        self._manual_timeout = float(
            self.get_parameter("manual_command_timeout").value
        )
        self._automatic_timeout = float(
            self.get_parameter("automatic_command_timeout").value
        )
        self._output_rate = float(self.get_parameter("output_rate").value)
        self._force_automatic_without_rc = bool(
            self.get_parameter("force_automatic_without_rc").value
        )

        if not 0 <= self._mode_index < 16:
            raise ValueError("mode_channel must be in the range 1..16")
        if not 0 < self._automatic_above_us < 3000:
            raise ValueError("automatic_above_us must be in 1..2999")
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self._rc_timeout,
                self._manual_timeout,
                self._automatic_timeout,
                self._output_rate,
            )
        ):
            raise ValueError("timeouts and output_rate must be positive")

        self._rc_connected = False
        self._automatic_mode = False
        self._last_rc_time = 0.0
        self._manual_command: PropulsionCommand | None = None
        self._automatic_command: PropulsionCommand | None = None
        self._last_manual_time = 0.0
        self._last_automatic_time = 0.0
        self._last_reported_mode: str | None = None

        self._command_publisher = self.create_publisher(
            PropulsionCommand, "/propulsion/command", 10
        )
        self._automatic_mode_publisher = self.create_publisher(
            Bool, "/control/automatic_mode", 10
        )
        self.create_subscription(
            CrsfChannels,
            "/rc/channels",
            self._rc_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PropulsionCommand,
            "/propulsion/manual_command",
            self._manual_callback,
            10,
        )
        self.create_subscription(
            PropulsionCommand,
            "/propulsion/automatic_command",
            self._automatic_callback,
            10,
        )
        self.create_timer(1.0 / self._output_rate, self._publish_selected)
        self.get_logger().info(
            f"Control mode: CH{self._mode_index + 1} <= "
            f"{self._automatic_above_us} us manual, > "
            f"{self._automatic_above_us} us automatic"
        )
        if self._force_automatic_without_rc:
            self.get_logger().warning(
                "BENCH TEST OVERRIDE: automatic mode forced without RC supervision"
            )

    def _rc_callback(self, message: CrsfChannels) -> None:
        self._last_rc_time = time.monotonic()
        self._rc_connected = bool(message.connected)
        self._automatic_mode = (
            int(message.microseconds[self._mode_index]) > self._automatic_above_us
        )

    def _manual_callback(self, message: PropulsionCommand) -> None:
        self._manual_command = message
        self._last_manual_time = time.monotonic()

    def _automatic_callback(self, message: PropulsionCommand) -> None:
        self._automatic_command = message
        self._last_automatic_time = time.monotonic()

    def _publish_selected(self) -> None:
        now = time.monotonic()
        rc_valid = (
            self._rc_connected
            and self._last_rc_time > 0.0
            and now - self._last_rc_time <= self._rc_timeout
        )
        selected = None
        mode = "rc_disconnected"
        automatic_selected = self._force_automatic_without_rc or (
            rc_valid and self._automatic_mode
        )
        if automatic_selected:
            mode = "automatic"
            if (
                self._automatic_command is not None
                and now - self._last_automatic_time <= self._automatic_timeout
            ):
                selected = self._automatic_command
            else:
                mode = "automatic_command_timeout"
        elif rc_valid:
            mode = "manual"
            if (
                self._manual_command is not None
                and now - self._last_manual_time <= self._manual_timeout
            ):
                selected = self._manual_command
            else:
                mode = "manual_command_timeout"

        output = PropulsionCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "control_mode_arbiter"
        if selected is None:
            output.channel_1_percent = 0.0
            output.channel_2_percent = 0.0
            output.source = mode
        else:
            output.channel_1_percent = selected.channel_1_percent
            output.channel_2_percent = selected.channel_2_percent
            output.source = f"{mode}:{selected.source or 'unspecified'}"
        self._command_publisher.publish(output)
        self._automatic_mode_publisher.publish(
            Bool(data=automatic_selected)
        )

        if mode != self._last_reported_mode:
            if "timeout" in mode or mode == "rc_disconnected":
                self.get_logger().warning(f"Control state: {mode}")
            else:
                self.get_logger().info(f"Control state: {mode}")
            self._last_reported_mode = mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ControlModeArbiter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
