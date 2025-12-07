# ppor_rotate_mapper/rotate_mapper_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import math, json, os

class RotateMapperNode(Node):
    def __init__(self):
        super().__init__("rotate_mapper")

        # 센서 방향 (rad)
        self.sensor_offset = {
            "front": 0.0,
            "left":  math.pi/2,
            "right": -math.pi/2,
            "back":  math.pi,
        }

        self.ranges = {k: None for k in self.sensor_offset}
        self.yaw = 0.0
        self.points = []
        self.mapping_active = False

        # 구독
        self.create_subscription(Range, "/tof/front",
                                 lambda m: self.update_range("front", m), 10)
        self.create_subscription(Range, "/tof/left",
                                 lambda m: self.update_range("left", m), 10)
        self.create_subscription(Range, "/tof/right",
                                 lambda m: self.update_range("right", m), 10)
        self.create_subscription(Range, "/tof/back",
                                 lambda m: self.update_range("back", m), 10)
        self.create_subscription(Float32, "/imu/yaw", self.yaw_cb, 10)

        # 주기적으로 포인트 기록 (20Hz)
        self.timer = self.create_timer(0.05, self.record_points)

        # 시작/정지 서비스
        self.srv = self.create_service(SetBool, "/rotate_mapper/toggle",
                                       self.toggle_cb)

        self.get_logger().info("RotateMapperNode ready")

    def yaw_cb(self, msg: Float32):
        self.yaw = msg.data  # rad

    def update_range(self, key, msg: Range):
        # inf / 너무 큰 값 필터링
        if math.isinf(msg.range) or msg.range <= 0.0:
            self.ranges[key] = None
        else:
            self.ranges[key] = msg.range

    def record_points(self):
        if not self.mapping_active:
            return

        yaw = self.yaw
        for name, r in self.ranges.items():
            if r is None:
                continue

            angle = yaw + self.sensor_offset[name]
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            self.points.append({"x": x, "y": y})

    def toggle_cb(self, req, resp):
        if req.data:
            # start
            self.points = []
            self.mapping_active = True
            self.get_logger().info("Mapping START")
            resp.success = True
            resp.message = "mapping started"
        else:
            # stop
            self.mapping_active = False
            self.save_json()
            resp.success = True
            resp.message = "mapping stopped & saved"
        return resp

    def save_json(self):
        save_path = "/home/pi/map_points.json"
        try:
            with open(save_path, "w") as f:
                json.dump(self.points, f, indent=2)
            self.get_logger().info(f"Saved {len(self.points)} points to {save_path}")
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RotateMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
