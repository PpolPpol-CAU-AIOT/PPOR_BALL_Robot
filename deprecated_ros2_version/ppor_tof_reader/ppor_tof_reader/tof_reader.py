import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from .sensor_manager import ToFSensorManager

class ToFReaderNode(Node):
    def __init__(self):
        super().__init__("tof_reader")

        gpio_map = {
            "front": 17,
            "left": 27,
            "right": 22,
            "back": 23,
        }

        address_map = {
            "front": 0x30,
            "left": 0x31,
            "right": 0x32,
            "back": 0x33,
        }

        # 센서 초기화
        self.manager = ToFSensorManager(gpio_map, address_map)

        # 독립 토픽 생성
        self.tof_pub  = {
            "front": self.create_publisher(Range, "/tof/front", 10),
            "left": self.create_publisher(Range, "/tof/left", 10),
            "right": self.create_publisher(Range, "/tof/right", 10),
            "back": self.create_publisher(Range, "/tof/back", 10),
        }

        # 50ms 마다 콜백 실행
        self.timer = self.create_timer(0.05, self.publish_ranges)

    def publish_ranges(self):
        data = self.manager.read_all()

        for name, dist in data.items():
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"tof_{name}"
            msg.range = dist if dist is not None else float("inf")

            self.tof_pub[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToFReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
