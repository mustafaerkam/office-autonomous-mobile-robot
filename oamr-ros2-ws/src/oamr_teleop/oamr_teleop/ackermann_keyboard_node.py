"""Ackermann araci icin canli klavye teleop node'u.

Neden hiz + direksiyon acisi (v, delta) ile surulur, wz ile degil:
bir Ackermann araci yaw hizini dogrudan secemez. Direksiyon acisi mekanik olarak
sinirlidir ve yaw hizi ondan turer:

    wz = v * tan(delta) / wheelbase

Bu model iki seyi bedavaya getirir. Birincisi, minimum donus yaricapi
(R_min = wheelbase / tan(delta_max)) gercekci kalir. Ikincisi, v=0 iken wz
otomatik olarak 0 cikar; boylece firmware'in reddettigi "dururken don" komutunu
uretmek yapisal olarak imkansizdir. Park halinde direksiyon cevirmek araci
dondurmez -- ekranda aci gorunur ama arac donmez, ki fiziksel gercek budur.

Yayin standart geometry_msgs/Twist'tir. Joystick asamasinda joy_node +
teleop_twist_joy ayni /cmd_vel'e baglanir ve bu node'a hic dokunulmaz.
"""

import curses
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

DRIVE_FORWARD_KEYS = (ord("w"), ord("W"), curses.KEY_UP)
DRIVE_REVERSE_KEYS = (ord("s"), ord("S"), curses.KEY_DOWN)
STEER_LEFT_KEYS = (ord("a"), ord("A"), curses.KEY_LEFT)
STEER_RIGHT_KEYS = (ord("d"), ord("D"), curses.KEY_RIGHT)


class AckermannKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("oamr_ackermann_teleop")

        # Fiziksel parametreler config/vehicle.yaml ile ayni anlamdadir.
        self.declare_parameter("wheelbase_m", 0.170)
        self.declare_parameter("max_linear_speed_mps", 0.40)
        self.declare_parameter("linear_step_mps", 0.05)
        self.declare_parameter("max_steering_angle_rad", 0.52)
        self.declare_parameter("steering_step_rad", 0.05)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_hold_timeout_s", 0.25)
        self.declare_parameter("steering_returns_to_center", True)
        self.declare_parameter("linear_slew_mps2", 0.8)

        self.wheelbase = float(self.get_parameter("wheelbase_m").value)
        self.max_speed = float(self.get_parameter("max_linear_speed_mps").value)
        self.linear_step = float(self.get_parameter("linear_step_mps").value)
        self.max_steering = float(self.get_parameter("max_steering_angle_rad").value)
        self.steering_step = float(self.get_parameter("steering_step_rad").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.key_hold_timeout = float(self.get_parameter("key_hold_timeout_s").value)
        self.steering_centers = bool(self.get_parameter("steering_returns_to_center").value)
        self.linear_slew = float(self.get_parameter("linear_slew_mps2").value)

        # Kademeli model: bu iki deger tuslarla ayarlanan "ayar noktalari"dir,
        # yon tuslari bunlari uygular.
        self.speed_setting = min(0.15, self.max_speed)
        self.steering_setting = min(0.30, self.max_steering)

        self.current_speed = 0.0
        self.current_steering = 0.0

        self.last_forward_key = 0.0
        self.last_reverse_key = 0.0
        self.last_left_key = 0.0
        self.last_right_key = 0.0

        self.status_message = "Baslatildi. ARM icin 'e'."
        self.last_error = "-"
        self.firmware_state = "telemetry bekleniyor"
        self.measured = (0.0, 0.0)

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Telemetri BEST_EFFORT yayinlanir (bayat ornegi yeniden gondermenin anlami
        # yok). ROS 2'de BEST_EFFORT yayinci ile RELIABLE abone UYUMSUZDUR ve tek bir
        # mesaj bile akmaz; bu yuzden abone tarafi bridge'in profiliyle ayni olmalidir.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            DiagnosticArray, "diagnostics", self.on_diagnostics, sensor_qos)
        self.create_subscription(
            JointState, "joint_states", self.on_joint_states, sensor_qos)

        self.arm_client = self.create_client(Trigger, "oamr/arm")
        self.disarm_client = self.create_client(Trigger, "oamr/disarm")

    # --- ROS geri cagirmalari -------------------------------------------
    def on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if not status.name.startswith("oamr"):
                continue
            self.firmware_state = status.message
            for entry in status.values:
                if entry.key == "last_error":
                    self.last_error = entry.value

    def on_joint_states(self, message: JointState) -> None:
        if len(message.velocity) >= 2:
            self.measured = (message.velocity[0], message.velocity[1])

    # --- surus mantigi ---------------------------------------------------
    def target_speed(self, now: float) -> float:
        forward = (now - self.last_forward_key) < self.key_hold_timeout
        reverse = (now - self.last_reverse_key) < self.key_hold_timeout
        if forward and not reverse:
            return self.speed_setting
        if reverse and not forward:
            return -self.speed_setting
        return 0.0

    def target_steering(self, now: float) -> float:
        left = (now - self.last_left_key) < self.key_hold_timeout
        right = (now - self.last_right_key) < self.key_hold_timeout
        if left and not right:
            return self.steering_setting
        if right and not left:
            return -self.steering_setting
        # Direksiyon merkeze donmuyorsa son aci korunur (gercek direksiyon gibi).
        return 0.0 if self.steering_centers else self.current_steering

    def step(self, dt: float) -> Twist:
        now = time.monotonic()

        # Hizda rampa: ani sicramalar hem mekanigi hem H-koprusunu zorlar.
        goal_speed = self.target_speed(now)
        max_delta = self.linear_slew * dt
        difference = goal_speed - self.current_speed
        if abs(difference) <= max_delta:
            self.current_speed = goal_speed
        else:
            self.current_speed += math.copysign(max_delta, difference)

        self.current_steering = self.target_steering(now)

        message = Twist()
        message.linear.x = self.current_speed
        # Ackermann donusumu: v=0 iken tan(delta) ne olursa olsun wz sifir kalir.
        message.angular.z = (self.current_speed
                             * math.tan(self.current_steering) / self.wheelbase)
        self.publisher.publish(message)
        return message

    def emergency_stop(self) -> None:
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.last_forward_key = 0.0
        self.last_reverse_key = 0.0
        self.last_left_key = 0.0
        self.last_right_key = 0.0
        self.publisher.publish(Twist())

    def call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.status_message = f"{label} servisi hazir degil (bridge calisiyor mu?)"
            return
        # call_async kullanmak zorunludur: senkron cagri curses dongusunu bloklar
        # ve tus okuma durur. Sonuc diagnostics uzerinden zaten gorulur.
        client.call_async(Trigger.Request())
        self.status_message = f"{label} gonderildi."

    def minimum_turning_radius(self) -> float:
        if abs(self.steering_setting) < 1e-6:
            return float("inf")
        return self.wheelbase / math.tan(abs(self.steering_setting))


def draw(screen, node: AckermannKeyboardNode, twist: Twist) -> None:
    screen.erase()
    height, width = screen.getmaxyx()

    def put(row: int, column: int, text: str, attribute=curses.A_NORMAL) -> None:
        if 0 <= row < height:
            screen.addnstr(row, column, text, max(0, width - column - 1), attribute)

    rule = "-" * min(58, max(10, width - 2))

    put(0, 1, "OAMR  ACKERMANN TELEOP", curses.A_BOLD)
    put(1, 1, rule)

    put(2, 1, "AYAR NOKTALARI", curses.A_BOLD)
    put(3, 3, f"hiz ayari       = {node.speed_setting:6.2f} m/s"
              f"   (max {node.max_speed:.2f})")
    steering_degrees = math.degrees(node.steering_setting)
    put(4, 3, f"direksiyon ayari= {node.steering_setting:6.2f} rad"
              f" = {steering_degrees:5.1f} deg"
              f"   (max {math.degrees(node.max_steering):.0f})")
    radius = node.minimum_turning_radius()
    radius_text = "sonsuz (duz)" if math.isinf(radius) else f"{radius:.2f} m"
    put(5, 3, f"donus yaricapi  = {radius_text}")

    put(7, 1, "ANLIK KOMUT", curses.A_BOLD)
    put(8, 3, f"v   = {twist.linear.x:6.3f} m/s")
    put(9, 3, f"delta = {node.current_steering:6.3f} rad "
              f"= {math.degrees(node.current_steering):5.1f} deg")
    put(10, 3, f"wz  = {twist.angular.z:6.3f} rad/s  (v*tan(delta)/L ile turetildi)")

    if abs(twist.linear.x) < 1e-6 and abs(node.current_steering) > 1e-6:
        put(11, 3, "durur haldeyken direksiyon donmez -- once ileri git",
            curses.A_DIM)

    put(13, 1, "LOW-LEVEL", curses.A_BOLD)
    put(14, 3, f"durum      : {node.firmware_state}")
    put(15, 3, f"olculen L/R: {node.measured[0]:6.2f} / {node.measured[1]:6.2f} rad/s")
    error_attribute = curses.A_BOLD if node.last_error not in ("-", "") else curses.A_DIM
    put(16, 3, f"son hata   : {node.last_error}", error_attribute)

    put(18, 1, rule)
    put(19, 1, "w/s ileri-geri   a/d direksiyon   (basili tut)")
    put(20, 1, "+/- hiz ayari    [/] direksiyon ayari")
    put(21, 1, "e ARM   x DISARM   BOSLUK acil dur   q cikis")
    put(23, 1, node.status_message, curses.A_DIM)
    screen.refresh()


def run(screen, node: AckermannKeyboardNode) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)

    period = 1.0 / node.publish_rate
    last_tick = time.monotonic()
    twist = Twist()

    while rclpy.ok():
        # Ayni dongude hem tuslari hem ROS geri cagirmalarini isleriz; ayri thread
        # gerekmez ve curses ekrani tek yerden guncellenir.
        while True:
            key = screen.getch()
            if key == -1:
                break
            now = time.monotonic()

            if key in DRIVE_FORWARD_KEYS:
                node.last_forward_key = now
            elif key in DRIVE_REVERSE_KEYS:
                node.last_reverse_key = now
            elif key in STEER_LEFT_KEYS:
                node.last_left_key = now
            elif key in STEER_RIGHT_KEYS:
                node.last_right_key = now
            elif key in (ord("+"), ord("=")):
                node.speed_setting = min(node.max_speed,
                                         node.speed_setting + node.linear_step)
                node.status_message = f"hiz ayari {node.speed_setting:.2f} m/s"
            elif key in (ord("-"), ord("_")):
                node.speed_setting = max(0.0, node.speed_setting - node.linear_step)
                node.status_message = f"hiz ayari {node.speed_setting:.2f} m/s"
            elif key == ord("]"):
                node.steering_setting = min(node.max_steering,
                                            node.steering_setting + node.steering_step)
                node.status_message = (
                    f"direksiyon ayari {math.degrees(node.steering_setting):.0f} deg")
            elif key == ord("["):
                node.steering_setting = max(0.0,
                                            node.steering_setting - node.steering_step)
                node.status_message = (
                    f"direksiyon ayari {math.degrees(node.steering_setting):.0f} deg")
            elif key == ord(" "):
                node.emergency_stop()
                node.status_message = "ACIL DUR"
            elif key in (ord("e"), ord("E")):
                node.call_trigger(node.arm_client, "ARM")
            elif key in (ord("x"), ord("X")):
                node.emergency_stop()
                node.call_trigger(node.disarm_client, "DISARM")
            elif key in (ord("q"), ord("Q")):
                node.emergency_stop()
                return

        rclpy.spin_once(node, timeout_sec=0.0)

        now = time.monotonic()
        elapsed = now - last_tick
        if elapsed >= period:
            last_tick = now
            twist = node.step(elapsed)
            draw(screen, node, twist)
        else:
            time.sleep(min(0.005, period - elapsed))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannKeyboardNode()
    try:
        curses.wrapper(run, node)
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal geri verildikten sonra son bir sifir Twist gonderilir.
        try:
            node.publisher.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
