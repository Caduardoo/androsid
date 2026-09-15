import base64
import json
import socket
import threading

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSPolicyKind
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import (
    BatteryState,
    CompressedImage,
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
)


# Android device frame -> ROS REP-103 FLU (landscape orientation)
def android_to_flu(x, y, z):
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

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.gps_frame = self.get_parameter("gps_frame").value

        self.qos_overrides = QoSOverridingOptions(
            policy_kinds=(
                QoSPolicyKind.RELIABILITY,
                QoSPolicyKind.DURABILITY,
                QoSPolicyKind.HISTORY,
                QoSPolicyKind.DEPTH,
            )
        )
        self.pub_imu = self.create_publisher(
            Imu, "imu/data_raw", 10, qos_overriding_options=self.qos_overrides
        )
        self.pub_mag = self.create_publisher(
            MagneticField, "imu/mag", 10, qos_overriding_options=self.qos_overrides
        )
        self.pub_gps = self.create_publisher(
            NavSatFix, "gps/fix", 10, qos_overriding_options=self.qos_overrides
        )
        self.pub_battery = self.create_publisher(
            BatteryState, "battery_state", 10, qos_overriding_options=self.qos_overrides
        )

        self.pub_img = {}

        self._last_accel = None
        self._logged_provider = False

        self._stop = threading.Event()
        self._sock = None
        self._send_lock = threading.Lock()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop.set()

        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            self.get_logger().warn("Reader thread still running after 2s")

        return super().destroy_node()

    def _run(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to {self.host}:{self.port}")
                with socket.create_connection(
                    (self.host, self.port), timeout=10
                ) as sock:
                    sock.settimeout(None)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._sock = sock
                    self.get_logger().info("Connected!")

                    self.pub_img.clear()

                    self._consume(sock)

            except OSError as e:
                if self._stop.is_set():
                    break
                self.get_logger().warn(f"Connection failed: {e}; retrying in 2s")
                self._stop.wait(2.0)

            finally:
                self._sock = None

    def _consume(self, sock):
        stream = sock.makefile("r", encoding="utf-8", newline="\n")
        while not self._stop.is_set():
            line = stream.readline()
            if not line:
                raise ConnectionError("Stream closed")
            line = line.strip()
            if not line:
                continue

            sample = json.loads(line)
            kind = sample.get("s")
            if kind == "gps":
                self._on_gps(sample)
            elif kind == "accel":
                self._last_accel = sample["v"]
            elif kind == "gyro":
                self._on_imu(sample)
            elif kind == "mag":
                self._on_mag(sample)
            elif kind == "battery":
                self._on_battery(sample)
            elif kind == "frame":
                self._on_frame(sample["t"], sample.get("c", "default"), base64.b64decode(sample["d"]))

    def _on_imu(self, sample):
        if self._last_accel is None:
            return

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

        # -1 in the first element is the REP-145 for "no orientation estimate here"
        msg.orientation_covariance[0] = -1.0

        # TODO: Rough fixed covariances. Replace with values from a stationary Allan
        msg.angular_velocity_covariance[0] = 4e-4
        msg.angular_velocity_covariance[4] = 4e-4
        msg.angular_velocity_covariance[8] = 4e-4
        msg.linear_acceleration_covariance[0] = 4e-2
        msg.linear_acceleration_covariance[4] = 4e-2
        msg.linear_acceleration_covariance[8] = 4e-2

        self.pub_imu.publish(msg)

    def _on_mag(self, sample):
        msg = MagneticField()
        msg.header.stamp = to_ros_time(sample["t"])
        msg.header.frame_id = self.imu_frame

        mx, my, mz = android_to_flu(*sample["v"])
        msg.magnetic_field.x = float(mx)
        msg.magnetic_field.y = float(my)
        msg.magnetic_field.z = float(mz)

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
        msg.status.service = NavSatStatus.SERVICE_GPS if provider == "gps" else 0
        msg.latitude = float(sample["lat"])
        msg.longitude = float(sample["lon"])
        msg.altitude = float(sample["alt"])

        horiz = float(sample.get("acc", 0.0)) ** 2
        vert = float(sample.get("vacc", 0.0)) ** 2 or horiz
        msg.position_covariance[0] = horiz
        msg.position_covariance[4] = horiz
        msg.position_covariance[8] = vert
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_gps.publish(msg)

    def _on_frame(self, stamp_nanos, camera_name, jpeg):
        pub = self.pub_img.get(camera_name)
        if pub is None:
            pub = self.create_publisher(
                CompressedImage,
                f"camera/{camera_name}/image_raw/compressed",
                10,
                qos_overriding_options=self.qos_overrides,
            )
            self.pub_img[camera_name] = pub

        msg = CompressedImage()
        msg.header.stamp = to_ros_time(stamp_nanos)
        msg.header.frame_id = f"camera_{camera_name}_optical_frame"
        msg.format = "jpeg"
        msg.data = jpeg
        pub.publish(msg)

    def _on_battery(self, sample):
        msg = BatteryState()
        msg.header.stamp = to_ros_time(sample["t"])
        msg.voltage = float(sample.get("voltage", float("nan")))
        msg.temperature = float(sample.get("temperature", float("nan")))
        msg.current = float(sample.get("current", float("nan")))
        msg.percentage = float(sample.get("percentage", float("nan")))
        msg.present = bool(sample.get("present", True))

        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")

        statuses = {
            "unknown": BatteryState.POWER_SUPPLY_STATUS_UNKNOWN,
            "charging": BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            "discharging": BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            "not_charging": BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
            "full": BatteryState.POWER_SUPPLY_STATUS_FULL,
        }
        msg.power_supply_status = statuses.get(
            sample.get("status", "unknown"), BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )

        healths = {
            "unknown": BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
            "good": BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            "overheat": BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT,
            "dead": BatteryState.POWER_SUPPLY_HEALTH_DEAD,
            "overvoltage": BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE,
            "unspecified_failure": BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
            "cold": BatteryState.POWER_SUPPLY_HEALTH_COLD,
        }
        msg.power_supply_health = healths.get(
            sample.get("health", "unknown"), BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )

        technologies = {
            "nimh": BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH,
            "li-ion": BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
            "li-poly": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
            "life": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
            "nicd": BatteryState.POWER_SUPPLY_TECHNOLOGY_NICD,
            "limn": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIMN,
        }
        msg.power_supply_technology = technologies.get(
            sample.get("tech", "unknown"), BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )
        self.pub_battery.publish(msg)

    def send_command(self, cmd: str, params: dict = None) -> bool:
        if params is None:
            params = {}

        if self._sock is None:
            self.get_logger().warn("Cannot send command: TCP socket is not connected")
            return False

        payload = {"cmd": cmd}
        payload.update(params)
        cmd_bytes = (json.dumps(payload) + "\n").encode("utf-8")

        try:
            with self._send_lock:
                sock = self._sock
                if sock is None:
                    raise OSError("Socket disconnected")
                sock.sendall(cmd_bytes)
            return True
        except (OSError, AttributeError) as e:
            self.get_logger().error(f"Failed to send command '{cmd}': {e}")
            return False

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