"""ROS 2 telemetry bridge with constrained propulsion parameter controls."""

from __future__ import annotations

import asyncio
import json
import time
import faulthandler
import signal
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from .global_view import GlobalView
from usv_interfaces.msg import (
    BallDetection,
    BallTargetState,
    CrsfChannels,
    CrsfLinkStatistics,
    PropulsionCommand,
    PwmOutputState,
)


class UsvTelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("usv_web_monitor")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self.state: dict[str, Any] = {
            "link": {},
            "rc": {},
            "control": {},
            "propulsion": {},
            "safety": {},
            "system": {},
            "vision": {},
            "received": {},
        }
        self._last_cpu_sample: tuple[int, int] | None = None
        self._bridge = CvBridge()
        self._vision_jpeg: bytes | None = None
        self._vision_frame_id = 0
        self._pending_vision_image = None
        self._ros_loop_at = time.monotonic()
        self._ros_max_gap_ms = 0.0
        self._global_view = GlobalView()
        self.plan_replies = {}
        self.plan_request_pub = self.create_publisher(String, '/navigation/goal_request', 10)
        self.create_subscription(String, '/navigation/goal_reply', self._on_plan_reply, 10)
        # Display is a latest-state consumer, never a reliable backpressure
        # dependency of the control/UDP publishers.
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/vision/global_observation',
                                 lambda m: self._global_view.observation(m.data), latest_qos)
        self.create_subscription(String, '/navigation/status',
                                 lambda m: self._global_view.status(m.data), 10)
        self.create_subscription(String, '/mission/status',
                                 lambda m: self._global_view.mission_status(m.data), 10)

        self.create_subscription(
            CrsfLinkStatistics,
            "/rc/link_statistics",
            self._on_link,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CrsfChannels,
            "/rc/channels",
            self._on_channels,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/rc/connected", self._on_connected, 10)
        self.create_subscription(
            Bool, "/control/automatic_mode", self._on_mode, 10
        )
        self.create_subscription(
            PropulsionCommand, "/propulsion/command", self._on_command, 10
        )
        self.create_subscription(
            PwmOutputState, "/actuators/pwm_state", self._on_pwm_state, 10
        )
        self.create_subscription(
            BallDetection, "/ball/detection", self._on_ball_detection, 10
        )
        self.create_subscription(
            BallTargetState, "/ball/target_state", self._on_target_state, 10
        )
        self.create_subscription(
            Float32, "/ball/desired_yaw_rate", self._on_desired_yaw_rate, 10
        )
        self.create_subscription(
            Bool, "/ball/follow_active", self._on_follow_active, 10
        )
        self.create_subscription(
            Image, "/ball/debug_image", self._on_debug_image, latest_qos
        )
        self.create_timer(1.0, self._update_system)
        self._get_pwm_parameters = self.create_client(
            GetParameters, "/pwm_actuator/get_parameters"
        )
        self._set_pwm_parameters = self.create_client(
            SetParametersAtomically, "/pwm_actuator/set_parameters_atomically"
        )
        self._update_system()

    def get_propulsion_settings(self):
        request = GetParameters.Request()
        request.names = [
            "output_limit_percent",
            "channel_1_trim_percent",
            "channel_2_trim_percent",
        ]
        return self._get_pwm_parameters.call_async(request)

    def set_propulsion_settings(self, values: dict[str, float]):
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        return self._set_pwm_parameters.call_async(request)

    def pwm_parameter_services_ready(self) -> bool:
        return (
            self._get_pwm_parameters.service_is_ready()
            and self._set_pwm_parameters.service_is_ready()
        )

    def _touch(self, name: str) -> None:
        self.state["received"][name] = time.monotonic()

    def _on_link(self, msg: CrsfLinkStatistics) -> None:
        self.state["link"] = {
            "rssi1": int(msg.uplink_rssi_1_dbm),
            "rssi2": int(msg.uplink_rssi_2_dbm),
            "quality": int(msg.uplink_link_quality),
            "snr": int(msg.uplink_snr_db),
            "activeAntenna": int(msg.active_antenna),
            "rfMode": int(msg.rf_mode),
            "txPowerCode": int(msg.tx_power_code),
            "downlinkRssi": int(msg.downlink_rssi_dbm),
            "downlinkQuality": int(msg.downlink_link_quality),
            "downlinkSnr": int(msg.downlink_snr_db),
        }
        self._touch("link")

    def _on_channels(self, msg: CrsfChannels) -> None:
        channels = [int(value) for value in msg.microseconds]
        self.state["rc"].update(
            {
                "connected": bool(msg.connected),
                "frameRate": float(msg.frame_rate_hz),
                "validFrames": int(msg.valid_frame_count),
                "crcErrors": int(msg.crc_error_count),
                "channels": channels,
                "modeChannelUs": channels[7] if len(channels) > 7 else None,
                "estopChannelUs": channels[4] if len(channels) > 4 else None,
            }
        )
        self._touch("channels")

    def _on_connected(self, msg: Bool) -> None:
        self.state["rc"]["connected"] = bool(msg.data)
        self._touch("connected")

    def _on_mode(self, msg: Bool) -> None:
        self.state["control"]["automatic"] = bool(msg.data)
        self._touch("mode")

    def _on_command(self, msg: PropulsionCommand) -> None:
        self.state["control"]["source"] = str(msg.source)
        self.state["propulsion"].update(
            {
                "command": [
                    float(msg.channel_1_percent),
                    float(msg.channel_2_percent),
                ]
            }
        )
        self._touch("command")

    def _on_pwm_state(self, msg: PwmOutputState) -> None:
        self.state["propulsion"].update(
            {
                "requested": [float(value) for value in msg.requested_percent],
                "applied": [float(value) for value in msg.applied_percent],
                "pulseUs": [int(value) for value in msg.pulse_width_us],
                "angleDeg": [int(value) for value in msg.controller_angle_deg],
            }
        )
        self.state["control"]["source"] = str(msg.source)
        self.state["safety"] = {
            "watchdog": bool(msg.watchdog_active),
            "emergencyStop": bool(msg.emergency_stop_active),
            "dryRun": bool(msg.dry_run),
        }
        self._touch("pwm")

    def _on_ball_detection(self, msg: BallDetection) -> None:
        self.state["vision"].update(
            {
                "detected": bool(msg.detected),
                "classId": int(msg.class_id),
                "className": str(msg.class_name),
                "confidence": float(msg.confidence),
                "imageWidth": int(msg.image_width),
                "imageHeight": int(msg.image_height),
                "centerX": float(msg.center_x),
                "centerY": float(msg.center_y),
                "horizontalError": (float(msg.center_x) - 0.5) * 2.0,
                "areaRatio": float(msg.area_ratio),
                "inferenceMs": float(msg.inference_ms),
            }
        )
        self._touch("detection")

    def _on_target_state(self, msg: BallTargetState) -> None:
        state_names = {
            BallTargetState.LOST: "LOST",
            BallTargetState.TRACKING: "TRACKING",
            BallTargetState.PREDICTING: "PREDICTING",
        }
        self.state["vision"].update(
            {
                "targetValid": bool(msg.target_valid),
                "trackingState": state_names.get(int(msg.tracking_state), "UNKNOWN"),
                "bearingDeg": float(msg.bearing_rad) * 180.0 / 3.141592653589793,
                "bearingRateDegS": float(msg.bearing_rate_rad_s) * 180.0 / 3.141592653589793,
                "detectionAgeMs": float(msg.detection_age_sec) * 1000.0,
                "imuValid": bool(msg.imu_valid),
                "yawRateDegS": float(msg.yaw_rate_rad_s) * 180.0 / 3.141592653589793,
            }
        )
        self._touch("target")

    def _on_desired_yaw_rate(self, msg: Float32) -> None:
        self.state["vision"]["desiredYawRateDegS"] = (
            float(msg.data) * 180.0 / 3.141592653589793
        )
        self._touch("desiredYawRate")

    def _on_follow_active(self, msg: Bool) -> None:
        self.state["vision"]["followActive"] = bool(msg.data)
        self._touch("followActive")

    def _on_debug_image(self, msg: Image) -> None:
        # Do not encode inside the ROS/HTTP event loop. Replace, don't queue,
        # images if the camera is faster than the display encoder.
        self._pending_vision_image = msg

    def _encode_vision_image(self, msg: Image) -> bytes | None:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82]
            )
        except Exception as exc:
            self.get_logger().warning(f"Vision image conversion failed: {exc}")
            return
        if not ok:
            return
        return encoded.tobytes()

    async def vision_encode_loop(self) -> None:
        while rclpy.ok():
            started = time.monotonic()
            msg, self._pending_vision_image = self._pending_vision_image, None
            if msg is not None:
                jpeg = await asyncio.to_thread(self._encode_vision_image, msg)
                if jpeg is not None:
                    self._vision_jpeg = jpeg
                    self._vision_frame_id += 1
                    self._touch("visionFrame")
            await asyncio.sleep(max(0.001, 0.1 - (time.monotonic() - started)))

    def latest_vision_frame(self) -> tuple[int, bytes | None]:
        return self._vision_frame_id, self._vision_jpeg

    def _update_system(self) -> None:
        memory = self._read_memory()
        temperature = self._read_cpu_temperature()
        cpu_usage = self._read_cpu_usage()
        rail_voltage, total_current, total_power, power_rail = (
            self._read_module_power()
        )
        load = self._read_load()
        self.state["system"] = {
            **memory,
            "cpuTempC": temperature,
            "cpuUsage": cpu_usage,
            "moduleRailVoltageV": rail_voltage,
            "totalCurrentA": total_current,
            "totalPowerW": total_power,
            "powerRail": power_rail,
            **load,
        }
        self._touch("system")

    @staticmethod
    def _read_memory() -> dict[str, float | None]:
        values: dict[str, int] = {}
        try:
            for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
                key, raw = line.split(":", 1)
                values[key] = int(raw.strip().split()[0])
        except (OSError, ValueError, TypeError, UnicodeError, IndexError):
            return {"memoryTotalMb": None, "memoryUsedMb": None, "memoryPercent": None}

        total_kb = values.get("MemTotal", 0)
        available_kb = values.get("MemAvailable", 0)
        if total_kb <= 0:
            return {"memoryTotalMb": None, "memoryUsedMb": None, "memoryPercent": None}
        used_kb = max(0, total_kb - available_kb)
        return {
            "memoryTotalMb": round(total_kb / 1024, 1),
            "memoryUsedMb": round(used_kb / 1024, 1),
            "memoryPercent": round(used_kb / total_kb * 100, 1),
        }

    @staticmethod
    def _read_cpu_temperature() -> float | None:
        sensors: list[tuple[int, float]] = []
        thermal_root = Path("/sys/class/thermal")
        try:
            zones = list(thermal_root.glob("thermal_zone*"))
        except OSError:
            return None

        for zone in zones:
            try:
                label = (zone / "type").read_text(encoding="utf-8").strip().lower()
                raw = float((zone / "temp").read_text(encoding="utf-8").strip())
            # Some Jetson thermal-zone sysfs nodes intermittently return an
            # invalid/empty payload through the text wrapper (TypeError). One
            # broken sensor must not prevent the dashboard from starting.
            except (OSError, ValueError, TypeError, UnicodeError):
                continue
            value = raw / 1000.0 if abs(raw) > 500 else raw
            if not (-20 <= value <= 150):
                continue
            priority = 0
            if "cpu" in label:
                priority = 4
            elif "tj" in label or "junction" in label:
                priority = 3
            elif "soc" in label:
                priority = 2
            elif "gpu" in label:
                priority = 1
            sensors.append((priority, value))

        if not sensors:
            return None
        best_priority = max(priority for priority, _ in sensors)
        best_values = [value for priority, value in sensors if priority == best_priority]
        return round(max(best_values), 1)

    def _read_cpu_usage(self) -> float | None:
        try:
            first_line = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0]
            values = [int(value) for value in first_line.split()[1:]]
        except (OSError, ValueError, TypeError, UnicodeError, IndexError):
            return None
        if len(values) < 4:
            return None
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        previous = self._last_cpu_sample
        self._last_cpu_sample = (total, idle)
        if previous is None:
            return None
        total_delta = total - previous[0]
        idle_delta = idle - previous[1]
        if total_delta <= 0:
            return None
        usage = (1.0 - idle_delta / total_delta) * 100.0
        return round(max(0.0, min(100.0, usage)), 1)

    @staticmethod
    def _read_module_power(
    ) -> tuple[float | None, float | None, float | None, str | None]:
        """Read the Jetson module input rail, not the carrier's external DC jack.

        On Orin Nano the INA3221 VDD_IN channel is the post-regulation module
        rail (about 5 V). Its voltage times current matches tegrastats VDD_IN
        power; it does not expose the upstream 19 V adapter voltage.
        """
        preferred_labels = ("VDD_IN", "VIN_SYS_5V0", "VDDIN", "VIN")
        readings: list[tuple[int, float, float | None, float | None, str]] = []
        try:
            monitors = list(Path("/sys/class/hwmon").glob("hwmon*"))
        except OSError:
            return None, None, None, None

        for monitor in monitors:
            try:
                monitor_name = (monitor / "name").read_text(encoding="utf-8").strip().lower()
            except (OSError, TypeError, UnicodeError):
                monitor_name = ""
            if "ina" not in monitor_name:
                continue
            for voltage_path in monitor.glob("in*_input"):
                channel = voltage_path.stem.removeprefix("in").removesuffix("_input")
                try:
                    raw_voltage = float(
                        voltage_path.read_text(encoding="utf-8").strip()
                    )
                except (OSError, ValueError, TypeError, UnicodeError):
                    continue
                try:
                    label = (monitor / f"in{channel}_label").read_text(
                        encoding="utf-8"
                    ).strip()
                except (OSError, TypeError, UnicodeError):
                    label = f"IN{channel}"
                volts = (
                    raw_voltage / 1000.0
                    if abs(raw_voltage) > 100
                    else raw_voltage
                )
                amps = None
                current_path = monitor / f"curr{channel}_input"
                try:
                    raw_current = float(
                        current_path.read_text(encoding="utf-8").strip()
                    )
                    amps = (
                        raw_current / 1000.0
                        if abs(raw_current) > 10
                        else raw_current
                    )
                except (OSError, ValueError, TypeError, UnicodeError):
                    pass
                watts = volts * amps if amps is not None else None
                upper_label = label.upper()
                priority = next(
                    (
                        index
                        for index, preferred in enumerate(preferred_labels)
                        if preferred in upper_label
                    ),
                    len(preferred_labels),
                )
                readings.append((priority, volts, amps, watts, label))

        if not readings:
            return None, None, None, None
        readings.sort(key=lambda item: item[0])
        _, voltage, current, power, label = readings[0]
        return (
            round(voltage, 2),
            round(current, 3) if current is not None else None,
            round(power, 2) if power is not None else None,
            label,
        )

    @staticmethod
    def _read_load() -> dict[str, float | int | None]:
        try:
            load1 = float(Path("/proc/loadavg").read_text(encoding="utf-8").split()[0])
        except (OSError, ValueError, TypeError, UnicodeError, IndexError):
            load1 = None
        try:
            uptime_sec = int(float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0]))
        except (OSError, ValueError, TypeError, UnicodeError, IndexError):
            uptime_sec = None
        return {"load1": load1, "uptimeSec": uptime_sec}

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        ages = {
            key: round((now - value) * 1000)
            for key, value in self.state["received"].items()
        }
        return {
            "type": "telemetry",
            "serverTimeMs": round(time.time() * 1000),
            "link": dict(self.state["link"]),
            "rc": dict(self.state["rc"]),
            "control": dict(self.state["control"]),
            "propulsion": dict(self.state["propulsion"]),
            "safety": dict(self.state["safety"]),
            "system": dict(self.state["system"]),
            "vision": dict(self.state["vision"]),
            "ageMs": ages,
            "global": self._global_view.snapshot(),
        }


    def _on_plan_reply(self, msg):
        try:
            reply=json.loads(msg.data)
            if not isinstance(reply,dict) or not isinstance(reply.get('request_id'),str):return
            self.plan_replies[reply['request_id']]=reply
            while len(self.plan_replies)>32:self.plan_replies.pop(next(iter(self.plan_replies)))
        except (ValueError,TypeError):pass


class DashboardServer:
    def __init__(self, node: UsvTelemetryNode, web_dir: Path) -> None:
        self.node = node
        self.web_dir = web_dir
        self.clients: set[web.WebSocketResponse] = set()
        self.closing = False
        self.app = web.Application()
        self.app.router.add_get("/", self.index)
        self.app.router.add_get("/global", self.global_page)
        self.app.router.add_get("/api/navigation", self.navigation_data)
        self.app.router.add_post('/api/goal', self.select_goal)
        self.app.router.add_get("/styles.css", self.styles)
        self.app.router.add_get("/app.js", self.script)
        self.app.router.add_get("/ws", self.websocket)
        self.app.router.add_get("/healthz", self.health)
        self.app.router.add_get("/vision/stream.mjpg", self.vision_stream)
        self.app.router.add_get("/api/propulsion-settings", self.get_settings)
        self.app.router.add_post("/api/propulsion-settings", self.set_settings)
        self.app.router.add_static("/assets", self.web_dir, show_index=False)

    async def index(self, _: web.Request) -> web.FileResponse:
        return web.FileResponse(self.web_dir / "index.html")

    async def global_page(self, _: web.Request) -> web.FileResponse:
        return web.FileResponse(self.web_dir / 'global.html')

    async def navigation_data(self, _: web.Request) -> web.Response:
        return web.json_response(self.node._global_view.snapshot(), headers={'Cache-Control': 'no-store'})

    async def select_goal(self, request):
        import uuid
        self._validate_same_origin(request)
        try:
            raw=await request.text()
            if len(raw)>16000:raise ValueError('请求过大')
            data=json.loads(raw)
            if not isinstance(data,dict):raise ValueError('需要JSON对象')
            if data.get('goal_side') not in ('left','right'):raise ValueError('球门无效')
            request_id=uuid.uuid4().hex
            data['request_id']=request_id
            encoded=json.dumps(data,allow_nan=False)
        except (ValueError,TypeError) as e:
            return web.json_response(dict(ok=False,error=str(e)),status=400)
        if not self.node.plan_request_pub.get_subscription_count():
            return web.json_response(dict(ok=False,error='导航节点未连接'),status=503)
        self.node.plan_request_pub.publish(String(data=encoded))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            reply=self.node.plan_replies.pop(request_id,None)
            if reply is not None:return web.json_response(reply,status=200 if reply.get('ok') else 409)
            await asyncio.sleep(.05)
        return web.json_response(dict(ok=False,error='节点确认超时：请检查地图中已加载路线，不要切自动'),status=504)

    async def styles(self, _: web.Request) -> web.FileResponse:
        return web.FileResponse(self.web_dir / "styles.css")

    async def script(self, _: web.Request) -> web.FileResponse:
        return web.FileResponse(self.web_dir / "app.js")

    async def health(self, _: web.Request) -> web.Response:
        return web.json_response(
            {"ok": True, "clients": len(self.clients), "node": self.node.get_name(),
             "rosLoopAgeMs": round((time.monotonic()-self.node._ros_loop_at)*1000),
             "rosMaxGapMs": round(self.node._ros_max_gap_ms, 1),
             "dataAgeMs": self.node.snapshot()['ageMs']}
        )

    async def vision_stream(self, request: web.Request) -> web.StreamResponse:
        response = web.StreamResponse(
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-store, no-cache, must-revalidate",
            }
        )
        await response.prepare(request)
        previous_id = -1
        try:
            while rclpy.ok() and not self.closing:
                frame_id, jpeg = self.node.latest_vision_frame()
                if jpeg is None or frame_id == previous_id:
                    await asyncio.sleep(0.05)
                    continue
                previous_id = frame_id
                await asyncio.wait_for(response.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode("ascii")
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                ), timeout=1.0)
        except (ConnectionResetError, asyncio.CancelledError, RuntimeError, asyncio.TimeoutError):
            pass
        return response

    @staticmethod
    async def _await_ros_future(future, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise web.HTTPGatewayTimeout(text="ROS parameter service timeout")
            await asyncio.sleep(0.01)
        exception = future.exception()
        if exception is not None:
            raise web.HTTPBadGateway(text=f"ROS parameter service failed: {exception}")
        return future.result()

    def _require_pwm_services(self) -> None:
        if not self.node.pwm_parameter_services_ready():
            raise web.HTTPServiceUnavailable(
                text="pwm_actuator parameter service is unavailable"
            )

    @staticmethod
    def _settings_from_response(response) -> dict[str, float]:
        names = (
            "output_limit_percent",
            "channel_1_trim_percent",
            "channel_2_trim_percent",
        )
        return {
            name: float(value.double_value)
            for name, value in zip(names, response.values)
        }

    async def get_settings(self, _: web.Request) -> web.Response:
        self._require_pwm_services()
        response = await self._await_ros_future(
            self.node.get_propulsion_settings()
        )
        return web.json_response(
            {"ok": True, "settings": self._settings_from_response(response)}
        )

    @staticmethod
    def _validate_same_origin(request: web.Request) -> None:
        origin = request.headers.get("Origin")
        if origin and urlsplit(origin).netloc != request.host:
            raise web.HTTPForbidden(text="cross-origin control request rejected")

    async def set_settings(self, request: web.Request) -> web.Response:
        self._validate_same_origin(request)
        self._require_pwm_services()
        try:
            payload = await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text="invalid JSON")
        limits = {
            "output_limit_percent": (0.0, 100.0),
            "channel_1_trim_percent": (-100.0, 100.0),
            "channel_2_trim_percent": (-100.0, 100.0),
        }
        if not isinstance(payload, dict) or set(payload) != set(limits):
            raise web.HTTPBadRequest(text="all three propulsion settings are required")
        values: dict[str, float] = {}
        for name, (minimum, maximum) in limits.items():
            raw = payload[name]
            if isinstance(raw, bool):
                raise web.HTTPBadRequest(text=f"{name} must be numeric")
            try:
                value = float(raw)
            except (TypeError, ValueError):
                raise web.HTTPBadRequest(text=f"{name} must be numeric")
            if not minimum <= value <= maximum:
                raise web.HTTPBadRequest(
                    text=f"{name} must be in {minimum:g}..{maximum:g}"
                )
            values[name] = value

        response = await self._await_ros_future(
            self.node.set_propulsion_settings(values)
        )
        if not response.result.successful:
            raise web.HTTPBadRequest(text=response.result.reason)
        confirmed = await self._await_ros_future(
            self.node.get_propulsion_settings()
        )
        self.node.get_logger().warning(
            "Web propulsion settings applied: "
            f"limit={values['output_limit_percent']:.1f}%, "
            f"trim=[{values['channel_1_trim_percent']:+.1f}%, "
            f"{values['channel_2_trim_percent']:+.1f}%]"
        )
        return web.json_response(
            {"ok": True, "settings": self._settings_from_response(confirmed)}
        )

    async def websocket(self, request: web.Request) -> web.WebSocketResponse:
        socket = web.WebSocketResponse(heartbeat=15)
        await socket.prepare(request)
        self.clients.add(socket)
        await socket.send_str(json.dumps(self.node.snapshot(), ensure_ascii=False))
        try:
            async for message in socket:
                if message.type == WSMsgType.TEXT and message.data == "ping":
                    await socket.send_str('{"type":"pong"}')
                elif message.type == WSMsgType.ERROR:
                    break
        finally:
            self.clients.discard(socket)
        return socket

    async def broadcast_loop(self) -> None:
        while rclpy.ok():
            if self.clients:
                payload = json.dumps(self.node.snapshot(), ensure_ascii=False)
                # A non-reading laptop must not freeze updates to all others.
                await asyncio.gather(*(self._send_telemetry(socket, payload)
                                       for socket in tuple(self.clients)))
            await asyncio.sleep(0.1)

    async def _send_telemetry(self, socket, payload) -> None:
        try:
            await asyncio.wait_for(socket.send_str(payload), timeout=0.5)
        except (ConnectionError, RuntimeError, asyncio.TimeoutError):
            self.clients.discard(socket)
            try:
                await asyncio.wait_for(socket.close(), timeout=0.2)
            except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                pass


async def _spin(node: Node) -> None:
    while rclpy.ok():
        now = time.monotonic()
        node._ros_max_gap_ms = max(node._ros_max_gap_ms, (now-node._ros_loop_at)*1000)
        node._ros_loop_at = now
        # Drain ready callbacks in a bounded burst; one callback per 5 ms
        # caps the ENTIRE dashboard at 200 callbacks/s, not 200 per topic.
        deadline = now + .002
        for _ in range(32):
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except ExternalShutdownException:
                return
            if time.monotonic() >= deadline:
                break
        await asyncio.sleep(0.002)


async def _hang_watchdog() -> None:
    while rclpy.ok():
        # Runs from Python's independent fault-handler watchdog, even if an
        # event-loop callback blocks and no HTTP request can get through.
        faulthandler.dump_traceback_later(10.0)
        await asyncio.sleep(1.0)


async def _run(node: UsvTelemetryNode) -> None:
    web_dir = Path(get_package_share_directory("usv_web_monitor")) / "web"
    dashboard = DashboardServer(node, web_dir)
    runner = web.AppRunner(dashboard.app)
    await runner.setup()

    bind_address = str(node.get_parameter("bind_address").value)
    port = int(node.get_parameter("port").value)
    site = web.TCPSite(runner, bind_address, port)
    await site.start()
    node.get_logger().info(
        f"USV dashboard ready at http://{bind_address}:{port}"
    )

    spin_task = asyncio.create_task(_spin(node))
    broadcast_task = asyncio.create_task(dashboard.broadcast_loop())
    encode_task = asyncio.create_task(node.vision_encode_loop())
    watchdog_task = asyncio.create_task(_hang_watchdog())
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    # Avoid injecting KeyboardInterrupt into an rclpy callback coroutine.
    # The signal requests orderly cancellation and closes HTTP/MJPEG streams.
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    stop_task = asyncio.create_task(stop.wait())
    tasks = (spin_task, broadcast_task, encode_task, watchdog_task, stop_task)
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
    finally:
        dashboard.closing = True
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        faulthandler.cancel_dump_traceback_later()
        for socket in tuple(dashboard.clients):
            try:
                await asyncio.wait_for(socket.close(), timeout=.2)
            except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                pass
        await runner.cleanup()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)


def main(args: list[str] | None = None) -> None:
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=args)
    node = UsvTelemetryNode()
    try:
        asyncio.run(_run(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
