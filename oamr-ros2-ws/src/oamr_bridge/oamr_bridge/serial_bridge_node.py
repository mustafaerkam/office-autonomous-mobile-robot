"""ROS 2 ile ESP32 low-level firmware arasindaki seri kopru.

Gorevi bilerek dardir: /cmd_vel'i V1 komutuna cevirmek, sabit hizda gondermek ve
firmware'den gelen ACK/ERR/TEL satirlarini ROS tarafina tasimak. Kinematik hesap
firmware'de, surus karari teleop tarafindadir; bu node ikisinin arasinda yalniz
tasiyicidir.

Iki ayri zaman asimi vardir ve ikisi de kasitlidir:
  - cmd_vel_timeout: ROS tarafi susarsa bridge sifir Twist gondermeye baslar.
  - firmware'in kendi 500 ms'i: bridge tamamen olurse firmware kendi durur.
Bridge sabit hizda gonderdigi icin ikincisi normalde hic tetiklenmez; o bir
son savunma hattidir.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import serial

from oamr_bridge import protocol


class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("oamr_serial_bridge")

        # Geometri parametresi bilerek yoktur: bu node kinematik hesap yapmaz,
        # Twist'i oldugu gibi firmware'e iletir ve donusumu firmware yapar.
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.4)
        self.declare_parameter("left_wheel_joint", "rear_left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "rear_right_wheel_joint")

        self.serial_port_name = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.command_rate = float(self.get_parameter("command_rate_hz").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.left_joint = self.get_parameter("left_wheel_joint").value
        self.right_joint = self.get_parameter("right_wheel_joint").value

        self.sequence = 0
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.reader = protocol.LineReader()
        self.last_telemetry = None
        self.last_error = ""
        self.error_count = 0
        self.ack_count = 0

        try:
            self.serial = serial.Serial(self.serial_port_name, self.baudrate, timeout=0)
        except serial.SerialException as error:
            self.get_logger().error(
                f"Seri port acilamadi ({self.serial_port_name}): {error}")
            raise

        self.get_logger().info(
            f"Seri kopru acik: {self.serial_port_name} @ {self.baudrate}, "
            f"{self.command_rate:.0f} Hz komut hizi")

        # Telemetry latched degil; en son ornek yeterlidir, gecmis tutulmaz.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.joint_publisher = self.create_publisher(JointState, "joint_states", sensor_qos)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", sensor_qos)

        self.create_service(Trigger, "oamr/arm", self.on_arm)
        self.create_service(Trigger, "oamr/disarm", self.on_disarm)

        self.create_timer(1.0 / self.command_rate, self.on_command_tick)
        self.create_timer(0.01, self.on_read_tick)
        self.create_timer(0.5, self.on_diagnostics_tick)

    # --- ROS girisleri ---------------------------------------------------
    def on_cmd_vel(self, message: Twist) -> None:
        self.last_cmd = message
        self.last_cmd_time = self.get_clock().now()

    def next_sequence(self) -> int:
        # 32-bit sinirinda sarmalamak firmware'in %lu alanini tasirmaz.
        self.sequence = (self.sequence + 1) % 0xFFFFFFFF
        return self.sequence

    def write(self, payload: bytes) -> None:
        try:
            self.serial.write(payload)
        except serial.SerialException as error:
            self.get_logger().error(f"Seri yazma hatasi: {error}")

    def on_command_tick(self) -> None:
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_vel_timeout:
            # ROS tarafi sustu; firmware'in kendi zaman asimini beklemeden sifirla.
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = float(self.last_cmd.linear.x)
            angular_z = float(self.last_cmd.angular.z)

        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.get_logger().warn("cmd_vel sonlu olmayan deger icerdi; sifir gonderildi.")
            linear_x = 0.0
            angular_z = 0.0

        self.write(protocol.encode_twist(self.next_sequence(), linear_x, angular_z))

    # --- firmware cikisi -------------------------------------------------
    def on_read_tick(self) -> None:
        try:
            waiting = self.serial.in_waiting
            data = self.serial.read(waiting) if waiting else b""
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f"Seri okuma hatasi: {error}")
            return

        for line in self.reader.feed(data):
            response = protocol.parse_line(line)
            if response is None:
                continue
            self.handle_response(response)

    def handle_response(self, response) -> None:
        if response.kind == "TEL":
            self.last_telemetry = response.telemetry
            self.publish_joint_states(response.telemetry)
            return

        if response.kind == "ERR":
            self.error_count += 1
            self.last_error = response.name or "UNKNOWN"
            # INVALID_KINEMATICS ve NOT_ARMED beklenen kullanici hatalaridir;
            # surekli hata seli olusturmamak icin throttle ile loglanir.
            self.get_logger().warn(
                f"Firmware ERR seq={response.sequence} {response.name}",
                throttle_duration_sec=1.0)
            return

        if response.kind == "ACK":
            self.ack_count += 1
            return

        if response.kind in ("HELLO", "PONG", "CAL"):
            self.get_logger().info(f"Firmware: {response.raw}")
            return

        # Firmware'in insan-okunur Turkce satirlari (acilis mesaji gibi). Makine
        # tarafi bunlara dayanmaz; bring-up sirasinda gormek icin debug seviyesinde
        # loglanir, ayri bir topic'e yayinlanmaz.
        self.get_logger().debug(f"Firmware metni: {response.raw}")

    def publish_joint_states(self, telemetry) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.left_joint, self.right_joint]
        # Firmware count'u isaret duzeltilmis olarak verir; radyana cevirmek icin
        # CPR gerekir ve CPR firmware tarafinda tutulur. Bu yuzden pozisyon simdilik
        # bos birakilir; hiz alani dogrudan olculen rad/s'dir.
        message.velocity = [
            telemetry.left_measured_rad_s,
            telemetry.right_measured_rad_s,
        ]
        self.joint_publisher.publish(message)

    def on_diagnostics_tick(self) -> None:
        status = DiagnosticStatus()
        status.name = "oamr: low-level baglantisi"
        status.hardware_id = self.serial_port_name

        telemetry = self.last_telemetry
        if telemetry is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Firmware'den henuz telemetry alinmadi"
        elif not telemetry.feedback_ready:
            status.level = DiagnosticStatus.WARN
            status.message = "Kalibrasyon eksik (CPR / SIGN / GAINS)"
        elif not telemetry.armed:
            status.level = DiagnosticStatus.OK
            status.message = "Hazir, DISARM"
        elif telemetry.cmd_timeout:
            status.level = DiagnosticStatus.WARN
            status.message = "ARM fakat komut zaman asiminda"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ARM ve komut aliniyor"

        if telemetry is not None:
            status.values = [
                KeyValue(key="armed", value=str(telemetry.armed)),
                KeyValue(key="feedback_ready", value=str(telemetry.feedback_ready)),
                KeyValue(key="cmd_timeout", value=str(telemetry.cmd_timeout)),
                KeyValue(key="left_measured_rad_s", value=f"{telemetry.left_measured_rad_s:.3f}"),
                KeyValue(key="right_measured_rad_s", value=f"{telemetry.right_measured_rad_s:.3f}"),
                KeyValue(key="steering_rad", value=f"{telemetry.steering_rad:.3f}"),
                KeyValue(key="left_pwm", value=str(telemetry.left_pwm)),
                KeyValue(key="right_pwm", value=str(telemetry.right_pwm)),
                KeyValue(key="last_error", value=self.last_error or "-"),
                KeyValue(key="error_count", value=str(self.error_count)),
            ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    # --- servisler -------------------------------------------------------
    def on_arm(self, request, response):
        self.write(protocol.encode_arm(self.next_sequence()))
        response.success = True
        response.message = "ARM gonderildi (sonucu diagnostics/ERR ile dogrulayin)"
        return response

    def on_disarm(self, request, response):
        self.write(protocol.encode_disarm(self.next_sequence()))
        response.success = True
        response.message = "DISARM gonderildi"
        return response

    def shutdown(self) -> None:
        # Cikarken motorun son hedefte kalmamasi icin acikca durdur.
        try:
            self.write(protocol.encode_stop(self.next_sequence()))
            self.write(protocol.encode_disarm(self.next_sequence()))
            self.serial.flush()
            self.serial.close()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except serial.SerialException:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
