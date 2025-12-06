import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan

class ToFScanNode(Node):
    def __init__(self):
        super().__init__("tof_scan")

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = self.declare_parameter("angle_increment", 0.05).get_parameter_value().double_value
        self.range_max = self.declare_parameter("range_max", 2.0).get_parameter_value().double_value

        self.num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        self.sensor_angles = {
            "front": 0.0,
            "left": math.pi / 2.0,
            "right": -math.pi / 2.0,
            "back": math.pi,
        }

        self.ranges = {
            "front": None,
            "left": None,
            "right": None,
            "back": None,
        }

        self.create_subscription(Range, "/tof/front", lambda msg: self.range_cb("front", msg), 10)
        self.create_subscription(Range, "/tof/left", lambda msg: self.range_cb("left", msg), 10)
        self.create_subscription(Range, "/tof/right", lambda msg: self.range_cb("right", msg), 10)
        self.create_subscription(Range, "/tof/back", lambda msg: self.range_cb("back", msg), 10)

        self.timer = self.create_timer(0.1, self.publish_scan)

    def range_cb(self, key, msg: Range):
        self.ranges[key] = msg.range

    def publish_scan(self):
        now = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "base_link"  # 또는 tof_front 등, TF 설계에 따라 조정

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = 0.02
        scan.range_max = self.range_max

        ranges = [float("inf")] * self.num_points

        for key, r in self.ranges.items():
            if r is None:
                continue
            angle = self.sensor_angles[key]
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.num_points:
                ranges[idx] = r

        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = ToFScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
