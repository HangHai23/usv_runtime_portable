#!/usr/bin/env python3
"""ROS 2 node for an ExpressLRS receiver using the CRSF UART protocol."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
from std_msgs.msg import Bool
from usv_interfaces.msg import CrsfChannels, CrsfLinkStatistics

from .crsf_protocol import (
    FRAME_TYPE_LINK_STATISTICS,
    FRAME_TYPE_RC_CHANNELS_PACKED,
    LinkStatistics,
    StreamParser,
    decode_channels,
)


class CrsfReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("crsf_receiver")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 420000)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("frame_timeout", 0.5)
        self.declare_parameter("frame_id", "crsf_receiver")

        self._port_name = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._frame_timeout = float(self.get_parameter("frame_timeout").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        if self._publish_rate <= 0.0 or self._frame_timeout <= 0.0:
            raise ValueError("publish_rate and frame_timeout must be greater than zero")

        self._serial = serial.Serial(
            port=self._port_name,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            exclusive=True,
        )
        self._serial.reset_input_buffer()
        self._parser = StreamParser()
        self._latest_raw: list[int] | None = None
        self._latest_microseconds: list[int] | None = None
        self._last_rc_monotonic = 0.0
        self._rc_frame_count = 0
        self._rate_sample_count = 0
        self._rate_sample_time = time.monotonic()
        self._frame_rate_hz = 0.0
        self._was_connected = False

        self._channels_publisher = self.create_publisher(
            CrsfChannels, "/rc/channels", qos_profile_sensor_data
        )
        self._link_publisher = self.create_publisher(
            CrsfLinkStatistics, "/rc/link_statistics", qos_profile_sensor_data
        )
        self._connected_publisher = self.create_publisher(Bool, "/rc/connected", 10)

        self.create_timer(0.002, self._read_serial)
        self.create_timer(1.0 / self._publish_rate, self._publish_channels)
        self.create_timer(0.5, self._publish_connection_state)
        self.get_logger().info(
            f"CRSF receiver opened on {self._port_name} at {self._baudrate} baud"
        )

    def _read_serial(self) -> None:
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting if waiting > 0 else 1)
        except serial.SerialException as exc:
            self.get_logger().fatal(f"Serial read failed: {exc}")
            raise

        now = time.monotonic()
        for frame in self._parser.feed(data):
            frame_type = frame[2]
            payload = frame[3:-1]
            if frame_type == FRAME_TYPE_RC_CHANNELS_PACKED:
                try:
                    self._latest_raw, self._latest_microseconds = decode_channels(payload)
                except ValueError as exc:
                    self.get_logger().warning(str(exc))
                    continue
                self._last_rc_monotonic = now
                self._rc_frame_count += 1
            elif frame_type == FRAME_TYPE_LINK_STATISTICS:
                try:
                    self._publish_link_statistics(LinkStatistics.decode(payload))
                except ValueError as exc:
                    self.get_logger().warning(str(exc))

        elapsed = now - self._rate_sample_time
        if elapsed >= 1.0:
            self._frame_rate_hz = (
                self._rc_frame_count - self._rate_sample_count
            ) / elapsed
            self._rate_sample_count = self._rc_frame_count
            self._rate_sample_time = now

    def _connected(self) -> bool:
        return (
            self._last_rc_monotonic > 0.0
            and time.monotonic() - self._last_rc_monotonic <= self._frame_timeout
        )

    def _publish_channels(self) -> None:
        if self._latest_raw is None or self._latest_microseconds is None:
            return
        message = CrsfChannels()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.raw = self._latest_raw
        message.microseconds = self._latest_microseconds
        message.frame_rate_hz = float(self._frame_rate_hz)
        message.valid_frame_count = self._parser.valid_frames
        message.crc_error_count = self._parser.crc_errors
        message.connected = self._connected()
        self._channels_publisher.publish(message)

    def _publish_link_statistics(self, statistics: LinkStatistics) -> None:
        message = CrsfLinkStatistics()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.uplink_rssi_1_dbm = statistics.uplink_rssi_1_dbm
        message.uplink_rssi_2_dbm = statistics.uplink_rssi_2_dbm
        message.uplink_link_quality = statistics.uplink_link_quality
        message.uplink_snr_db = statistics.uplink_snr_db
        message.active_antenna = statistics.active_antenna
        message.rf_mode = statistics.rf_mode
        message.tx_power_code = statistics.tx_power_code
        message.downlink_rssi_dbm = statistics.downlink_rssi_dbm
        message.downlink_link_quality = statistics.downlink_link_quality
        message.downlink_snr_db = statistics.downlink_snr_db
        self._link_publisher.publish(message)

    def _publish_connection_state(self) -> None:
        connected = self._connected()
        self._connected_publisher.publish(Bool(data=connected))
        if connected != self._was_connected:
            if connected:
                self.get_logger().info("CRSF RC link connected")
            else:
                self.get_logger().warning("CRSF RC link lost")
            self._was_connected = connected

    def destroy_node(self):
        if self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CrsfReceiverNode()
        rclpy.spin(node)
    except serial.SerialException as exc:
        print(f"CRSF receiver startup failed: {exc}")
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
