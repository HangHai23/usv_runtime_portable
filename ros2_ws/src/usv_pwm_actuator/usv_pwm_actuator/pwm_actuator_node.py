#!/usr/bin/env python3
"""Convert final propulsion percentages into dual UART PWM commands."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
import serial
import smbus
from usv_interfaces.msg import CrsfChannels, PropulsionCommand, PwmOutputState

from .pwm_protocol import (
    apply_output_limit_and_trim,
    build_uart_command,
    percent_to_compensated_pulse_us,
    pulse_us_to_controller_angle,
)


class PwmActuatorNode(Node):
    def __init__(self) -> None:
        super().__init__("pwm_actuator")
        self.declare_parameter("backend", "i2c")
        self.declare_parameter("port", "/dev/ttyTHS2")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_address", 0x2D)
        self.declare_parameter("channel_1", 1)
        self.declare_parameter("channel_2", 2)
        self.declare_parameter("output_rate", 10.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("inter_command_delay", 0.05)
        # Conservative fallback when no YAML is loaded. Project launch files
        # load the global limit from config/pwm_actuator.yaml.
        self.declare_parameter("output_limit_percent", 100.0)
        self.declare_parameter("channel_1_trim_percent", 0.0)
        self.declare_parameter("channel_2_trim_percent", 0.0)
        self.declare_parameter("channel_1_forward_start_us", 1500)
        self.declare_parameter("channel_2_forward_start_us", 1500)
        self.declare_parameter("channel_1_reverse_start_us", 1500)
        self.declare_parameter("channel_2_reverse_start_us", 1500)
        self.declare_parameter("emergency_stop_enabled", True)
        self.declare_parameter("emergency_stop_channel", 5)
        self.declare_parameter("emergency_stop_threshold_us", 1500)
        self.declare_parameter("emergency_stop_rc_timeout", 0.5)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("forward_only", False)
        self.declare_parameter("unidirectional_equal_pulse_increment", False)
        self.declare_parameter("channel_1_unidirectional_start_us", 1000)
        self.declare_parameter("channel_2_unidirectional_start_us", 1000)
        self._forward_only = bool(self.get_parameter("forward_only").value)
        self._equal_pulse_increment = bool(
            self.get_parameter("unidirectional_equal_pulse_increment").value
        )
        self._unidirectional_start = tuple(int(self.get_parameter(
            f"channel_{i}_unidirectional_start_us").value) for i in (1, 2))
        if any(not 1000 <= v < 2000 for v in self._unidirectional_start):
            raise ValueError('unidirectional start pulses must be in 1000..1999')

        self._backend = str(self.get_parameter("backend").value).lower()
        self._port_name = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._i2c_bus_number = int(self.get_parameter("i2c_bus").value)
        self._i2c_address = int(self.get_parameter("i2c_address").value)
        self._channels = (
            int(self.get_parameter("channel_1").value),
            int(self.get_parameter("channel_2").value),
        )
        self._output_rate = float(self.get_parameter("output_rate").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._inter_command_delay = float(self.get_parameter("inter_command_delay").value)
        self._output_limit_percent = float(
            self.get_parameter("output_limit_percent").value
        )
        self._channel_trim_percent = (
            float(self.get_parameter("channel_1_trim_percent").value),
            float(self.get_parameter("channel_2_trim_percent").value),
        )
        self._forward_start_us = (
            int(self.get_parameter("channel_1_forward_start_us").value),
            int(self.get_parameter("channel_2_forward_start_us").value),
        )
        self._reverse_start_us = (
            int(self.get_parameter("channel_1_reverse_start_us").value),
            int(self.get_parameter("channel_2_reverse_start_us").value),
        )
        self._emergency_stop_enabled = bool(
            self.get_parameter("emergency_stop_enabled").value
        )
        self._emergency_stop_index = (
            int(self.get_parameter("emergency_stop_channel").value) - 1
        )
        self._emergency_stop_threshold_us = int(
            self.get_parameter("emergency_stop_threshold_us").value
        )
        self._emergency_stop_rc_timeout = float(
            self.get_parameter("emergency_stop_rc_timeout").value
        )
        self._dry_run = bool(self.get_parameter("dry_run").value)

        if self._backend not in ("i2c", "uart"):
            raise ValueError("backend must be 'i2c' or 'uart'")
        if not 0 <= self._i2c_address <= 0x7F:
            raise ValueError("i2c_address must be a 7-bit I2C address")
        if self._i2c_bus_number < 0:
            raise ValueError("i2c_bus cannot be negative")
        if any(channel < 1 or channel > 16 for channel in self._channels):
            raise ValueError("channel_1 and channel_2 must be in the range 1..16")
        if self._channels[0] == self._channels[1]:
            raise ValueError("channel_1 and channel_2 must be different")
        if self._output_rate <= 0.0 or self._command_timeout <= 0.0:
            raise ValueError("output_rate and command_timeout must be positive")
        if self._inter_command_delay < 0.0:
            raise ValueError("inter_command_delay cannot be negative")
        if not math.isfinite(self._output_limit_percent) or not (
            0.0 <= self._output_limit_percent <= 100.0
        ):
            raise ValueError("output_limit_percent must be in the range 0..100")
        self._validate_unidirectional_limit(self._output_limit_percent)
        if any(
            not math.isfinite(trim) or not -100.0 <= trim <= 100.0
            for trim in self._channel_trim_percent
        ):
            raise ValueError("channel trim parameters must be in the range -100..100")
        if any(not 1500 <= value <= 2000 for value in self._forward_start_us):
            raise ValueError("forward start pulse parameters must be in 1500..2000 us")
        if any(not 1000 <= value <= 1500 for value in self._reverse_start_us):
            raise ValueError("reverse start pulse parameters must be in 1000..1500 us")
        if not 0 <= self._emergency_stop_index < 16:
            raise ValueError("emergency_stop_channel must be in the range 1..16")
        if not 0 < self._emergency_stop_threshold_us < 3000:
            raise ValueError("emergency_stop_threshold_us must be in 1..2999")
        if self._emergency_stop_rc_timeout <= 0.0:
            raise ValueError("emergency_stop_rc_timeout must be positive")
        if (
            self._backend == "uart"
            and self._port_name == "/dev/ttyTHS1"
            and not self._dry_run
        ):
            raise ValueError(
                "/dev/ttyTHS1 is reserved for the 420000-baud CRSF receiver; "
                "use a separate UART for the 9600-baud PWM controller"
            )

        self.add_on_set_parameters_callback(self._parameter_update_callback)

        self._serial = None
        self._i2c_bus = None
        if not self._dry_run and self._backend == "uart":
            self._serial = serial.Serial(
                port=self._port_name,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                exclusive=True,
            )
        elif not self._dry_run:
            self._i2c_bus = smbus.SMBus(self._i2c_bus_number)

        self._requested = [0.0, 0.0]
        self._source = "startup"
        self._last_command_time = 0.0
        self._watchdog_active = True
        # Fail safe at startup: propulsion is permitted only after a fresh CH5
        # low-position frame has been received.
        self._emergency_stop_active = self._emergency_stop_enabled
        self._last_rc_time = 0.0

        self._state_publisher = self.create_publisher(
            PwmOutputState, "/actuators/pwm_state", 10
        )
        self.create_subscription(
            PropulsionCommand,
            "/propulsion/command",
            self._command_callback,
            10,
        )
        if self._emergency_stop_enabled:
            self.create_subscription(
                CrsfChannels,
                "/rc/channels",
                self._rc_channels_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(1.0 / self._output_rate, self._update_output)

        self._write_outputs([0.0, 0.0])
        if self._dry_run:
            mode = f"DRY RUN/{self._backend}"
        elif self._backend == "uart":
            mode = f"UART {self._port_name} @ {self._baudrate}"
        else:
            mode = f"I2C /dev/i2c-{self._i2c_bus_number} @ 0x{self._i2c_address:02X}"
        self.get_logger().info(f"PWM actuator ready ({mode}); outputs initialized to neutral")
        self.get_logger().warning(f'ESC forward_only={self._forward_only}; stop pulse={1000 if self._forward_only else 1500} us')
        self.get_logger().info(
            f"Output limit={self._output_limit_percent:.1f}%; "
            f"trim=[{self._channel_trim_percent[0]:+.1f}%, "
            f"{self._channel_trim_percent[1]:+.1f}%]"
        )
        self.get_logger().info(
            "ESC start compensation: "
            f"forward={list(self._forward_start_us)} us; "
            f"reverse={list(self._reverse_start_us)} us"
        )
        if self._emergency_stop_enabled:
            self.get_logger().info(
                f"Bottom-level emergency stop: CH{self._emergency_stop_index + 1} "
                f"> {self._emergency_stop_threshold_us} us; release < "
                f"{self._emergency_stop_threshold_us} us"
            )

    def _command_callback(self, message: PropulsionCommand) -> None:
        values = [float(message.channel_1_percent), float(message.channel_2_percent)]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected propulsion command containing NaN or infinity")
            return
        self._requested = values
        self._source = message.source or "unspecified"
        self._last_command_time = time.monotonic()

    def _parameter_update_callback(self, parameters) -> SetParametersResult:
        new_limit = self._output_limit_percent
        new_trims = list(self._channel_trim_percent)
        try:
            for parameter in parameters:
                if parameter.name == "output_limit_percent":
                    new_limit = float(parameter.value)
                    if not math.isfinite(new_limit) or not 0.0 <= new_limit <= 100.0:
                        raise ValueError("output_limit_percent must be in 0..100")
                elif parameter.name == "channel_1_trim_percent":
                    new_trims[0] = float(parameter.value)
                elif parameter.name == "channel_2_trim_percent":
                    new_trims[1] = float(parameter.value)
            if any(
                not math.isfinite(trim) or not -100.0 <= trim <= 100.0
                for trim in new_trims
            ):
                raise ValueError("channel trim parameters must be in -100..100")
            self._validate_unidirectional_limit(new_limit)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        changed = (
            new_limit != self._output_limit_percent
            or tuple(new_trims) != self._channel_trim_percent
        )
        self._output_limit_percent = new_limit
        self._channel_trim_percent = tuple(new_trims)
        if changed:
            self.get_logger().warning(
                f"Runtime output settings changed: limit={new_limit:.1f}%; "
                f"trim=[{new_trims[0]:+.1f}%, {new_trims[1]:+.1f}%]"
            )
        return SetParametersResult(successful=True)

    def _engage_emergency_stop(self, reason: str) -> None:
        if not self._emergency_stop_active:
            self._emergency_stop_active = True
            self.get_logger().error(f"EMERGENCY STOP engaged: {reason}")
            # Do not wait for the periodic output timer.
            self._write_outputs([0.0, 0.0])
            self._publish_state([0.0, 0.0])

    def _rc_channels_callback(self, message: CrsfChannels) -> None:
        self._last_rc_time = time.monotonic()
        if not message.connected:
            self._engage_emergency_stop("RC link unavailable")
            return
        pulse_us = int(message.microseconds[self._emergency_stop_index])
        if pulse_us > self._emergency_stop_threshold_us:
            self._engage_emergency_stop(
                f"CH{self._emergency_stop_index + 1}={pulse_us} us"
            )
        elif (
            pulse_us < self._emergency_stop_threshold_us
            and self._emergency_stop_active
        ):
            self._emergency_stop_active = False
            self.get_logger().warning(
                f"Emergency stop released: CH{self._emergency_stop_index + 1}="
                f"{pulse_us} us"
            )

    def _update_output(self) -> None:
        if self._emergency_stop_enabled and (
            self._last_rc_time == 0.0
            or time.monotonic() - self._last_rc_time > self._emergency_stop_rc_timeout
        ):
            self._engage_emergency_stop("RC channel data timeout")
        age = time.monotonic() - self._last_command_time
        watchdog = self._last_command_time == 0.0 or age > self._command_timeout
        applied = (
            [0.0, 0.0]
            if watchdog or self._emergency_stop_active
            else [
                apply_output_limit_and_trim(
                    self._requested[index],
                    self._output_limit_percent,
                    self._channel_trim_percent[index],
                )
                for index in range(2)
            ]
        )
        if watchdog and not self._watchdog_active:
            self.get_logger().warning("Propulsion command timeout; outputs returned to neutral")
        self._watchdog_active = watchdog
        if self._forward_only:
            applied = [max(0.0, v) for v in applied]
        self._write_outputs(applied)
        self._publish_state(applied)

    def _write_outputs(self, applied: list[float]) -> None:
        for index, percent in enumerate(applied):
            pulse_us = self._pulse_us(index, percent)
            angle = pulse_us_to_controller_angle(pulse_us)
            command = build_uart_command(self._channels[index], angle)
            if self._i2c_bus is not None:
                self._i2c_bus.write_byte_data(
                    self._i2c_address, self._channels[index], angle
                )
            elif self._serial is not None:
                self._serial.write(command)
                self._serial.flush()
            if index == 0 and self._inter_command_delay > 0.0:
                time.sleep(self._inter_command_delay)

    def _pulse_us(self, index: int, percent: float) -> int:
        if self._forward_only:
            if percent <= 0.0 or self._output_limit_percent <= 0.0:
                return 1000
            maximum = 1000.0 + 10.0 * self._output_limit_percent
            start = min(float(self._unidirectional_start[index]), maximum)
            if self._equal_pulse_increment:
                common_span = min(maximum - min(self._unidirectional_start),
                                  2000.0 - max(self._unidirectional_start))
                maximum = start + common_span
            return round(start + (maximum-start)*min(percent/self._output_limit_percent, 1.0))
        return percent_to_compensated_pulse_us(
            percent,
            self._output_limit_percent,
            self._forward_start_us[index],
            self._reverse_start_us[index],
        )

    def _validate_unidirectional_limit(self, limit: float) -> None:
        if not self._forward_only or not self._equal_pulse_increment or limit <= 0.0:
            return
        # Common span saturates at the calibrated 2000us hardware endpoint.
        # Both channels retain equal increments above their individual starts.
        if max(self._unidirectional_start) >= 2000:
            raise ValueError('startup threshold must leave a positive PWM range')

    def _publish_state(self, applied: list[float]) -> None:
        message = PwmOutputState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "pwm_controller"
        message.requested_percent = self._requested
        message.applied_percent = applied
        message.pulse_width_us = [self._pulse_us(i, v) for i, v in enumerate(applied)]
        message.controller_angle_deg = [
            pulse_us_to_controller_angle(v) for v in message.pulse_width_us
        ]
        message.watchdog_active = self._watchdog_active
        message.emergency_stop_active = self._emergency_stop_active
        message.dry_run = self._dry_run
        message.source = self._source
        self._state_publisher.publish(message)

    def destroy_node(self):
        try:
            self._write_outputs([0.0, 0.0])
        finally:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
            if self._i2c_bus is not None:
                self._i2c_bus.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PwmActuatorNode()
        rclpy.spin(node)
    except (OSError, serial.SerialException, ValueError) as exc:
        print(f"PWM actuator startup failed: {exc}")
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
