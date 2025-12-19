# ppor_rotate_mapper/rotate_mapper_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import math, json, os, statistics


class RotateMapperNode(Node):
    def __init__(self):
        super().__init__("rotate_mapper")

        # FLRB 방향 정의
        self.sensor_offset = {
            "front": 0.0,
            "left":  math.pi/2,
            "right": -math.pi/2,
            "back":  math.pi,
        }

        self.ranges = {k: None for k in self.sensor_offset}
        self.yaw = 0.0

        # raw samples
        self.samples = {k: [] for k in self.sensor_offset}

        self.mapping_active = False

        # ToF 센서 구독
        self.create_subscription(Range, "/tof/front",
                                 lambda m: self.update_range("front", m), 10)
        self.create_subscription(Range, "/tof/left",
                                 lambda m: self.update_range("left", m), 10)
        self.create_subscription(Range, "/tof/right",
                                 lambda m: self.update_range("right", m), 10)
        self.create_subscription(Range, "/tof/back",
                                 lambda m: self.update_range("back", m), 10)

        # IMU yaw
        self.create_subscription(Float32, "/imu/yaw", self.yaw_cb, 10)

        # 20Hz로 샘플 수집
        self.timer = self.create_timer(0.05, self.record_samples)

        # rotate_mapper toggle 서비스
        self.srv = self.create_service(
            SetBool, "/rotate_mapper/toggle", self.toggle_cb
        )

        self.save_path = "/home/pi/PPOR_BALL/scan_current.json"

        self.get_logger().info("RotateMapperNode ready (FLRB scan w/ median)")

    # ---------------------------------------------
    # Callbacks
    # ---------------------------------------------
    def yaw_cb(self, msg: Float32):
        self.yaw = msg.data

    def update_range(self, key, msg: Range):
        if math.isinf(msg.range) or msg.range <= 0.02:
            self.ranges[key] = None
        else:
            self.ranges[key] = msg.range

    # ---------------------------------------------
    # Sample recording (20Hz)
    # ---------------------------------------------
    def record_samples(self):
        if not self.mapping_active:
            return

        for name, r in self.ranges.items():
            if r is None:
                continue
            self.samples[name].append(r)

    # ---------------------------------------------
    # Service start/stop scan
    # ---------------------------------------------
    def toggle_cb(self, req, resp):
        if req.data:
            # start mapping
            self.mapping_active = True
            self.samples = {k: [] for k in self.sensor_offset}
            self.get_logger().info("Mapping START (collecting ToF samples)")
            resp.success = True
            resp.message = "mapping started"
        else:
            # stop mapping
            self.mapping_active = False
            self.save_scan()
            resp.success = True
            resp.message = "mapping stopped & saved"
        return resp

    # ---------------------------------------------
    # Save scan_current.json (median-based)
    # ---------------------------------------------
    def save_scan(self):
        summary = {}
        raw = {}

        for name, vals in self.samples.items():
            raw[name] = vals
            valid = [v for v in vals if v > 0.02]

            if len(valid) == 0:
                summary[name] = None
            else:
                summary[name] = float(statistics.median(valid))

        data = {
            "front": summary.get("front"),
            "left": summary.get("left"),
            "right": summary.get("right"),
            "back": summary.get("back"),
            "raw": raw,
        }

        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, "w") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(
                f"Saved scan (median) to {self.save_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RotateMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
