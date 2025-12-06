import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class AppBridgeNode(Node):
    def __init__(self):
        super().__init__("app_bridge")
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.get_logger().info("AppBridgeNode started (placeholder)")

    def odom_cb(self, msg: Odometry):
        # TODO: 여기서 추후 WebSocket/TCP로 앱에 전송
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AppBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
