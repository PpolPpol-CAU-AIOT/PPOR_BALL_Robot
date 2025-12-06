import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import serial
import re

ODOM_PATTERN = re.compile(
    r"<ODOM,L:(?P<L>-?\d+),R:(?P<R>-?\d+),Y:(?P<Y>-?\d+\.?\d*),D:(?P<D>\d+)>"
)

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
            self.get_logger().warn(
                f"Failed to open serial port {port} @ {baud}: {e}"
            )
            self.serial_connected = False
            self.ser = None

        self.pub_left = self.create_publisher(Int32, "/wheel/left_ticks", 10)
        self.pub_right = self.create_publisher(Int32, "/wheel/right_ticks", 10)
        self.pub_yaw = self.create_publisher(Float32, "/imu/yaw", 10)
        self.pub_front = self.create_publisher(Range, "/tof/front", 10)

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.01, self.read_serial)

        self.front_frame = self.declare_parameter("front_frame", "tof_front").get_parameter_value().string_value

    def read_serial(self):
        if not self.serial_connected:
            return
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not line:
            return

        match = ODOM_PATTERN.match(line)
        if not match:
            return

        left = int(match.group("L"))
        right = int(match.group("R"))
        yaw = float(match.group("Y"))
        dist_mm = int(match.group("D"))

        msg_l = Int32()
        msg_l.data = left
        self.pub_left.publish(msg_l)

        msg_r = Int32()
        msg_r.data = right
        self.pub_right.publish(msg_r)

        msg_y = Float32()
        msg_y.data = yaw
        self.pub_yaw.publish(msg_y)

        front_msg = Range()
        front_msg.header.stamp = self.get_clock().now().to_msg()
        front_msg.header.frame_id = self.front_frame
        front_msg.radiation_type = Range.INFRARED
        front_msg.field_of_view = 0.05
        front_msg.min_range = 0.02
        front_msg.max_range = 2.0
        front_msg.range = dist_mm / 1000.0
        self.pub_front.publish(front_msg)

    def cmd_vel_callback(self, msg: Twist):
        if not self.serial_connected:
            return
        lin = msg.linear.x
        ang = msg.angular.z
        cmd_str = f"<CMD,LIN:{lin:.3f},ANG:{ang:.3f}>\n"
        try:
            self.ser.write(cmd_str.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
