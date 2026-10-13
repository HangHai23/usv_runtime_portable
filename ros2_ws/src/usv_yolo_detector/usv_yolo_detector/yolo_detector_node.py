#!/usr/bin/env python3
"""Run portable ONNX CPU or optional TensorRT inference on the latest ROS image."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from usv_interfaces.msg import BallDetection
from .model_path import resolve_engine_path


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")
        self.declare_parameter("engine_path", "")
        self.declare_parameter("onnx_threads", 2)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detection_topic", "/ball/detection")
        self.declare_parameter("debug_image_topic", "/ball/debug_image")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("maximum_detections", 20)
        self.declare_parameter("device", "0")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("model_retry_interval", 5.0)

        self._engine_path = resolve_engine_path(str(self.get_parameter("engine_path").value))
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._debug_topic = str(self.get_parameter("debug_image_topic").value)
        self._target_class_id = int(self.get_parameter("target_class_id").value)
        self._confidence = float(
            self.get_parameter("confidence_threshold").value
        )
        self._iou = float(self.get_parameter("iou_threshold").value)
        self._image_size = int(self.get_parameter("image_size").value)
        self._maximum_detections = int(
            self.get_parameter("maximum_detections").value
        )
        self._device = str(self.get_parameter("device").value)
        self._publish_debug = bool(
            self.get_parameter("publish_debug_image").value
        )
        self._retry_interval = float(
            self.get_parameter("model_retry_interval").value
        )
        if not 0.0 <= self._confidence <= 1.0 or not 0.0 <= self._iou <= 1.0:
            raise ValueError("confidence_threshold and iou_threshold must be in 0..1")
        if self._image_size <= 0 or self._maximum_detections <= 0:
            raise ValueError("image_size and maximum_detections must be positive")
        if self._retry_interval <= 0.0:
            raise ValueError("model_retry_interval must be positive")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._bridge = CvBridge()
        self._detection_publisher = self.create_publisher(
            BallDetection, self._detection_topic, 10
        )
        self._debug_publisher = self.create_publisher(
            Image, self._debug_topic, sensor_qos
        )
        self._latest_lock = threading.Lock()
        self._latest_condition = threading.Condition(self._latest_lock)
        self._latest_image = None
        self._latest_header = None
        self._stop = threading.Event()
        self._model = None
        self._model_error_reported = False
        self._subscription = self.create_subscription(
            Image, self._image_topic, self._on_image, sensor_qos
        )
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        if self._engine_path:
            self.get_logger().info(f"Configured YOLO model: {self._engine_path}")
        else:
            self.get_logger().warning(
                "engine_path is empty; set it in config/yolo_detector.yaml and restart"
            )
        self.get_logger().info(
            f"Subscribing {self._image_topic}; publishing {self._detection_topic}"
        )

    def destroy_node(self):
        self._stop.set()
        with self._latest_condition:
            self._latest_condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=5.0)
        return super().destroy_node()

    def _on_image(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        with self._latest_condition:
            # Replace, rather than queue, old frames so inference stays real-time.
            self._latest_image = image
            self._latest_header = message.header
            self._latest_condition.notify()

    def _load_model(self) -> bool:
        if not self._engine_path:
            return False
        path = Path(self._engine_path).expanduser()
        if path.suffix.lower() not in (".engine", ".plan", ".onnx"):
            if not self._model_error_reported:
                self.get_logger().error("engine_path must point to .onnx, .engine or .plan")
                self._model_error_reported = True
            return False
        if not path.is_file():
            if not self._model_error_reported:
                self.get_logger().warning(f"TensorRT engine not found: {path}")
                self._model_error_reported = True
            return False
        try:
            if path.suffix.lower() == '.onnx':
                from .onnx_backend import OnnxBackend
                threads = int(self.get_parameter('onnx_threads').value)
                if not 1 <= threads <= 4:
                    raise ValueError('onnx_threads must be in 1..4')
                self._model = OnnxBackend(str(path), threads=threads)
            else:
                from ultralytics import YOLO
                self._model = YOLO(str(path), task="detect")
        except Exception as exc:
            if not self._model_error_reported:
                self.get_logger().error(f"Failed to load TensorRT engine: {exc}")
                self._model_error_reported = True
            return False
        self._model_error_reported = False
        self.get_logger().info(f"YOLO model loaded ({'ONNX CPU' if path.suffix.lower() == '.onnx' else 'TensorRT'}): {path}")
        return True

    def _inference_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            if self._model is None:
                if not self._load_model():
                    self._stop.wait(self._retry_interval)
                    continue
            with self._latest_condition:
                self._latest_condition.wait_for(
                    lambda: self._latest_image is not None or self._stop.is_set(),
                    timeout=1.0,
                )
                if self._stop.is_set():
                    break
                if self._latest_image is None:
                    continue
                image = self._latest_image
                header = self._latest_header
                self._latest_image = None
                self._latest_header = None
            started = time.perf_counter()
            try:
                results = self._model.predict(
                    source=image,
                    conf=self._confidence,
                    iou=self._iou,
                    imgsz=self._image_size,
                    max_det=self._maximum_detections,
                    device=self._device,
                    verbose=False,
                )
                inference_ms = (time.perf_counter() - started) * 1000.0
                self._publish_result(results[0], image, header, inference_ms)
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")
                self._model = None
                self._stop.wait(self._retry_interval)

    def _publish_result(self, result, image, header, inference_ms: float) -> None:
        image_height, image_width = image.shape[:2]
        message = BallDetection()
        message.header = header
        message.image_width = image_width
        message.image_height = image_height
        message.inference_ms = float(inference_ms)

        candidates = []
        boxes = result.boxes
        if boxes is not None:
            for index in range(len(boxes)):
                class_id = int(boxes.cls[index].item())
                if self._target_class_id >= 0 and class_id != self._target_class_id:
                    continue
                confidence = float(boxes.conf[index].item())
                candidates.append((confidence, index, class_id))

        if candidates:
            confidence, index, class_id = max(candidates, key=lambda item: item[0])
            x_min, y_min, x_max, y_max = (
                float(value) for value in boxes.xyxy[index].tolist()
            )
            width = max(0.0, x_max - x_min)
            height = max(0.0, y_max - y_min)
            message.detected = True
            message.class_id = class_id
            names = result.names
            message.class_name = str(names.get(class_id, class_id))
            message.confidence = confidence
            message.x_min = x_min
            message.y_min = y_min
            message.x_max = x_max
            message.y_max = y_max
            message.center_x = ((x_min + x_max) * 0.5) / image_width
            message.center_y = ((y_min + y_max) * 0.5) / image_height
            message.width = width / image_width
            message.height = height / image_height
            message.area_ratio = (width * height) / (image_width * image_height)
        else:
            message.detected = False
            message.class_id = -1
            message.class_name = ""
        self._detection_publisher.publish(message)

        if self._publish_debug:
            debug_message = self._bridge.cv2_to_imgmsg(result.plot(), encoding="bgr8")
            debug_message.header = header
            self._debug_publisher.publish(debug_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
