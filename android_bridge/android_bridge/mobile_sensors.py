#!/usr/bin/env python3

import json
import socket
import struct
import threading

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage, Imu, MagneticField, NavSatFix, NavSatStatus

TYPE_JSON = 0x01
TYPE_FRAME = 0x02

# Standard gravity, only used to sanity-check that accel units look like m/s^2.
G = 9.80665


def android_to_flu(x, y, z):
    """Android device frame -> ROS REP-103 FLU.

    Android's frame is fixed to the device's portrait orientation and
    does not follow screen rotation: X right, Y up the screen, Z out of the screen.

    Mounting: Device landscape, rear camera pointing forward, buttons up

        forward (x_ros) = -Z   rear camera points opposite the screen normal
        left    (y_ros) = +Y   top edge of the device points left
        up      (z_ros) = +X   right edge in portrait, where the buttons are
    """
    return -z, y, x


def to_ros_time(nanos):
    return Time(sec=int(nanos // 1_000_000_000), nanosec=int(nanos % 1_000_000_000))


class MobileSensors(Node):

    def __init__(self):
        super().__init__("mobile_sensors")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 9870)
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("gps_frame", "gps_link")
        self.declare_parameter("camera_frame", "camera_optical_frame")

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.gps_frame = self.get_parameter("gps_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", sensor_qos)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", sensor_qos)
        self.pub_gps = self.create_publisher(NavSatFix, "gps/fix", sensor_qos)
        self.pub_img = self.create_publisher(
            CompressedImage, "camera/image_raw/compressed", sensor_qos
        )

        # Accelerometer and gyroscope arrive as separate events; hold the most
        # recent accel and emit a combined Imu when the gyro sample lands.
        self._last_accel = None
        self._warned_units = False
        self._logged_provider = False

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()

    def _run(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.get_logger().info(f"connecting to {self.host}:{self.port}")
                with socket.create_connection(
                    (self.host, self.port), timeout=10
                ) as sock:
                    sock.settimeout(None)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info("connected")
                    self._consume(sock)
            except OSError as exc:
                self.get_logger().warn(f"connection failed: {exc}; retrying in 2s")
                self._stop.wait(2.0)

    @staticmethod
    def _read_exactly(sock, count):
        chunks = []
        remaining = count
        while remaining:
            chunk = sock.recv(remaining)
            if not chunk:
                raise ConnectionError("stream closed")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _consume(self, sock):
        while not self._stop.is_set():
            header = self._read_exactly(sock, 5)
            msg_type, length = struct.unpack(">BI", header)
            payload = self._read_exactly(sock, length)

            if msg_type == TYPE_JSON:
                self._on_json(json.loads(payload.decode("utf-8")))
            elif msg_type == TYPE_FRAME:
                stamp = struct.unpack(">q", payload[:8])[0]
                self._on_frame(stamp, payload[8:])
            else:
                self.get_logger().warn(f"unknown frame type {msg_type}, skipping")

    def _on_json(self, sample):
        kind = sample.get("s")
        if kind == "gps":
            self._on_gps(sample)
        elif kind == "accel":
            self._last_accel = sample["v"]
        elif kind == "gyro":
            self._on_imu(sample)
        elif kind == "mag":
            self._on_mag(sample)

    def _on_imu(self, sample):
        if self._last_accel is None:
            return  # Wait until we have an accel sample to pair with

        msg = Imu()
        msg.header.stamp = to_ros_time(sample["t"])
        msg.header.frame_id = self.imu_frame

        gx, gy, gz = android_to_flu(*sample["v"])
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        ax, ay, az = android_to_flu(*self._last_accel)
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # -1 in the first element is the REP-145 convention for "no orientation
        # estimate here". Run imu_filter_madgwick downstream to produce one.
        msg.orientation_covariance[0] = -1.0

        # Rough fixed covariances. Replace with values from a stationary Allan
        # variance run if you care about the output of a real estimator.
        msg.angular_velocity_covariance[0] = 4e-4
        msg.angular_velocity_covariance[4] = 4e-4
        msg.angular_velocity_covariance[8] = 4e-4
        msg.linear_acceleration_covariance[0] = 4e-2
        msg.linear_acceleration_covariance[4] = 4e-2
        msg.linear_acceleration_covariance[8] = 4e-2

        self._check_units(self._last_accel)
        self.pub_imu.publish(msg)

    def _on_mag(self, sample):
        msg = MagneticField()
        msg.header.stamp = to_ros_time(sample["t"])
        msg.header.frame_id = self.imu_frame

        # Android reports microtesla, ROS wants tesla.
        mx, my, mz = android_to_flu(*sample["v"])
        msg.magnetic_field.x = float(mx) * 1e-6
        msg.magnetic_field.y = float(my) * 1e-6
        msg.magnetic_field.z = float(mz) * 1e-6

        self.pub_mag.publish(msg)

    def _on_gps(self, sample):
        provider = sample.get("prov", "gps")

        if not self._logged_provider:
            self._logged_provider = True
            self.get_logger().info(
                f"first fix from '{provider}', "
                f"horizontal accuracy {float(sample.get('acc', 0.0)):.1f} m"
            )

        msg = NavSatFix()
        msg.header.stamp = to_ros_time(sample["t"])
        msg.header.frame_id = self.gps_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        # `service` is a bitmask of the GNSS constellations used. A Wi-Fi or cell
        # trilaterated fix used none of them, so claiming SERVICE_GPS would lie to
        # anything downstream that branches on it. The covariance below still
        # carries the real uncertainty either way.
        msg.status.service = (
            NavSatStatus.SERVICE_GPS if provider == "gps" else 0
        )
        msg.latitude = float(sample["lat"])
        msg.longitude = float(sample["lon"])
        msg.altitude = float(sample["alt"])

        # Android accuracy is a 68% horizontal radius in metres; square it for
        # a variance and spread it over both horizontal axes.
        horiz = float(sample.get("acc", 0.0)) ** 2
        vert = float(sample.get("vacc", 0.0)) ** 2 or horiz
        msg.position_covariance[0] = horiz
        msg.position_covariance[4] = horiz
        msg.position_covariance[8] = vert
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_gps.publish(msg)

    def _on_frame(self, stamp_nanos, jpeg):
        msg = CompressedImage()
        msg.header.stamp = to_ros_time(stamp_nanos)
        msg.header.frame_id = self.camera_frame
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub_img.publish(msg)

    def _check_units(self, accel):
        if self._warned_units:
            return
        magnitude = sum(component**2 for component in accel) ** 0.5
        if not (0.5 * G < magnitude < 2.0 * G):
            self.get_logger().warn(
                f"accel magnitude {magnitude:.2f} is far from {G:.2f} m/s^2 -- "
                "check that the device is stationary and the axes are right"
            )
        self._warned_units = True


def main(args=None):
    rclpy.init(args=args)
    node = MobileSensors()
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
