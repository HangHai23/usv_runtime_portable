#!/usr/bin/env python3
import math
import os

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from YbImuLib import YbImuSerial


GRAVITY = 9.80665
MICROTESLA_TO_TESLA = 1.0e-6


class YbImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_driver")
        self.declare_parameter("port", "/dev/myimu")
        self.declare_parameter("fallback_port", "/dev/ttyUSB0")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("hardware_report_rate", 100)
        self.declare_parameter("publish_rate", 100.0)

        requested_port = str(self.get_parameter("port").value)
        fallback_port = str(self.get_parameter("fallback_port").value)
        self._port = requested_port if os.path.exists(requested_port) else fallback_port
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._fixed_frame = str(self.get_parameter("fixed_frame").value)
        hardware_rate = int(self.get_parameter("hardware_report_rate").value)
        rate = float(self.get_parameter("publish_rate").value)
        if not 10 <= hardware_rate <= 100:
            raise ValueError("hardware_report_rate must be in 10..100 Hz")
        if rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if rate > hardware_rate:
            raise ValueError("publish_rate cannot exceed hardware_report_rate")

        if not os.path.exists(self._port):
            raise FileNotFoundError(
                f"IMU serial port not found: tried {requested_port} and {fallback_port}"
            )

        self._imu = YbImuSerial(self._port, debug=False)
        # Do this on every startup: the device may not retain its report rate
        # after power cycling, and a fast ROS timer must not repeat stale data.
        self._imu.set_report_rate(hardware_rate)
        self._imu.create_receive_threading()
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)
        self._marker_pub = self.create_publisher(Marker, "/imu/marker", 10)
        self._baro_pub = self.create_publisher(Float32MultiArray, "/baro", 10)
        self._euler_pub = self.create_publisher(Float32MultiArray, "/euler", 10)
        self._tf = TransformBroadcaster(self)
        self._warned_bad_quaternion = False
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Yahboom IMU opened on {self._port}; hardware report "
            f"{hardware_rate} Hz, ROS publish {rate:.1f} Hz"
        )

    @staticmethod
    def _normalized_quaternion(w: float, x: float, y: float, z: float):
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if not math.isfinite(norm) or norm < 1.0e-6:
            return None
        return w / norm, x / norm, y / norm, z / norm

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        ax, ay, az = self._imu.get_accelerometer_data()
        gx, gy, gz = self._imu.get_gyroscope_data()
        mx, my, mz = self._imu.get_magnetometer_data()
        qw, qx, qy, qz = self._imu.get_imu_quaternion_data()
        height, temperature, pressure, pressure_contrast = self._imu.get_baro_data()
        roll, pitch, yaw = self._imu.get_imu_attitude_data(ToAngle=True)
        quaternion = self._normalized_quaternion(qw, qx, qy, qz)
        if quaternion is None:
            if not self._warned_bad_quaternion:
                self.get_logger().warning("Waiting for a valid quaternion from the IMU")
                self._warned_bad_quaternion = True
            return
        qw, qx, qy, qz = quaternion

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax * GRAVITY
        imu_msg.linear_acceleration.y = ay * GRAVITY
        imu_msg.linear_acceleration.z = az * GRAVITY
        imu_msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.01]
        imu_msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self._imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header
        mag_msg.magnetic_field.x = mx * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field.y = my * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field.z = mz * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field_covariance = [1.0e-10, 0.0, 0.0, 0.0, 1.0e-10, 0.0, 0.0, 0.0, 1.0e-10]
        self._mag_pub.publish(mag_msg)

        baro_msg = Float32MultiArray()
        baro_msg.data = [height, temperature, pressure, pressure_contrast]
        self._baro_pub.publish(baro_msg)

        euler_msg = Float32MultiArray()
        euler_msg.data = [roll, pitch, yaw]
        self._euler_pub.publish(euler_msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._fixed_frame
        transform.child_frame_id = self._frame_id
        transform.transform.rotation = imu_msg.orientation
        self._tf.sendTransform(transform)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._frame_id
        marker.ns = "ybimu"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.08
        marker.scale.z = 0.025
        marker.color.r = 0.12
        marker.color.g = 0.55
        marker.color.b = 0.95
        marker.color.a = 1.0
        self._marker_pub.publish(marker)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YbImuNode()
        rclpy.spin(node)
    except (FileNotFoundError, PermissionError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"IMU startup failed: {exc}")
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
