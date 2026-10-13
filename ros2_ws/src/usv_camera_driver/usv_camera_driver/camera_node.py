#!/usr/bin/env python3
"""ROS 2 USB camera publisher with automatic device discovery and reconnect."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("device", "auto")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("reconnect_interval", 1.0)
        self.declare_parameter("publish_rate", 30.0)

        self._device = str(self.get_parameter("device").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._fourcc = str(self.get_parameter("fourcc").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._topic = str(self.get_parameter("image_topic").value)
        self._reconnect_interval = float(
            self.get_parameter("reconnect_interval").value
        )
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        if self._width <= 0 or self._height <= 0:
            raise ValueError("camera width and height must be positive")
        if self._fps <= 0.0 or self._publish_rate <= 0.0:
            raise ValueError("camera fps and publish_rate must be positive")
        if self._reconnect_interval <= 0.0:
            raise ValueError("reconnect_interval must be positive")
        if len(self._fourcc) != 4:
            raise ValueError("fourcc must contain exactly four characters")

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(
            Image, self._topic, qos_profile_sensor_data
        )
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"USB camera waiting on {self._device}; requested "
            f"{self._width}x{self._height} @ {self._fps:.1f} FPS; publishing {self._topic}"
        )

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        return super().destroy_node()

    def _candidates(self) -> list[str]:
        if self._device != "auto":
            if self._device.isdigit():
                return [f"/dev/video{self._device}"]
            return [self._device]
        return [str(path) for path in sorted(Path("/dev").glob("video*"))]

    def _open_camera(self):
        for device in self._candidates():
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                continue
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            capture.set(cv2.CAP_PROP_FPS, self._fps)
            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                return capture, device, frame
            capture.release()
        return None, None, None

    def _capture_loop(self) -> None:
        capture = None
        active_device = None
        initial_frame = None
        last_wait_log = 0.0
        minimum_period = 1.0 / self._publish_rate
        while not self._stop.is_set() and rclpy.ok():
            if capture is None:
                capture, active_device, initial_frame = self._open_camera()
                if capture is None:
                    now = time.monotonic()
                    if now - last_wait_log >= 5.0:
                        self.get_logger().warning(
                            "No usable USB camera found; waiting for /dev/video*"
                        )
                        last_wait_log = now
                    self._stop.wait(self._reconnect_interval)
                    continue
                actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = capture.get(cv2.CAP_PROP_FPS)
                self.get_logger().info(
                    f"Camera connected: {active_device}, "
                    f"{actual_width}x{actual_height} @ {actual_fps:.1f} FPS"
                )

            started = time.monotonic()
            if initial_frame is not None:
                frame = initial_frame
                initial_frame = None
                ok = True
            else:
                ok, frame = capture.read()
            if not ok or frame is None or not frame.size:
                self.get_logger().warning(
                    f"Camera disconnected or frame read failed: {active_device}; rescanning"
                )
                capture.release()
                capture = None
                active_device = None
                continue

            message = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._frame_id
            self._publisher.publish(message)
            remaining = minimum_period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop.wait(remaining)

        if capture is not None:
            capture.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsbCameraNode()
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
