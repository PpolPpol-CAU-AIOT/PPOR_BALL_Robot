import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import serial
import re


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        port = self.declare_parameter("port", "/dev/ttyUSB0").get_parameter_value().string_value
        baud = self.declare_parameter("baudrate", 115200).get_parameter_value().integer_value
        try:
            self.get_logger().info(f"Opening serial port {port} @ {baud}")
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
            self.serial_connected = True
        except Exception as e:
            self.get_logger().error(f"[ArduinoBridge] Serial open failed: {e}")
            self.serial_connected = False
            self.ser = None

        self.wheel_base = 0.12

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )


    def cmd_vel_callback(self, msg: Twist):
        if not self.serial_connected:
            return

        v = msg.linear.x          # m/s
        w = msg.angular.z         # rad/s
        h = msg.linear.z
        W = self.wheel_base

        # differential drive
        left = v - (w * (W / 2))
        right = v + (w * (W / 2))

        # scale to -100 ~ +100 percent
        max_speed = 0.20  # 최대 물리 속도(m/s), 너 로봇에 맞게 조정 가능

        left_percent = int(max(min(left / max_speed * 100, 100), -100))
        right_percent = int(max(min(right / max_speed * 100, 100), -100))

        h_percent = int(max(min(h * 100, 100), -100))

        # format: R100,L80,
        cmd = f"R{right_percent},L{left_percent},H{h_percent},"

        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"[ArduinoBridge] Serial write error: {e}")

def manual_cmd_cb(self, msg: String):
    cmd = msg.data
    self.get_logger().info(f"[MANUAL] Received manual command: {cmd}")

    t = Twist()

    # ============================
    # 즉시 manual 제어로 전환
    # 모든 NAV FSM 정지
    # ============================
    self.nav_active = False
    self.state = "IDLE"

    if cmd == "front":
        t.linear.x = self.linear_speed

    elif cmd == "back":
        t.linear.x = -self.linear_speed

    elif cmd == "left":
        t.angular.z = self.angular_speed

    elif cmd == "right":
        t.angular.z = -self.angular_speed

    elif cmd == "stop":
        self.stop()
        return

    elif cmd == "dance":
        self.get_logger().info("[MANUAL] Starting DANCE MODE")
        import time

        for _ in range(2):
            # Left rotate
            t = Twist()
            t.angular.z = self.angular_speed
            self.cmd_pub.publish(t)
            time.sleep(0.5)

            # Right rotate
            t = Twist()
            t.angular.z = -self.angular_speed
            self.cmd_pub.publish(t)
            time.sleep(0.5)

            # Forward
            t = Twist()
            t.linear.x = self.linear_speed
            self.cmd_pub.publish(t)
            time.sleep(0.4)

            # Backward
            t = Twist()
            t.linear.x = -self.linear_speed
            self.cmd_pub.publish(t)
            time.sleep(0.4)

            t = Twist()
            t.linear.z = 1.0   # H +100%
            self.cmd_pub.publish(t)
            time.sleep(0.4)

            t = Twist()
            t.linear.z = -1.0  # H -100%
            self.cmd_pub.publish(t)
            time.sleep(0.4)

        self.stop()
        return

    self.cmd_pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()