import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus2
import time
import math

MPU_ADDR = 0x68

# 레지스터
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47

class MPU9250Gyro:
    def __init__(self, bus_id=1, addr=MPU_ADDR):
        self.bus = smbus2.SMBus(bus_id)
        self.addr = addr

        # 깨우기
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Gyro full-scale ±250 dps (0)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        time.sleep(0.1)

        # bias 측정
        self.bias_z = self.calibrate_bias()

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg+1)
        val  = (high << 8) | low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read_gyro_z_dps(self):
        raw = self.read_word(GYRO_ZOUT_H)
        # ±250 dps -> sensitivity 131 LSB/(°/s)
        dps = raw / 131.0
        dps -= self.bias_z

        if abs(dps) < 0.1:
            dps= 0.0
        return dps

    def calibrate_bias(self, samples=500):
        print("[IMU] Calibrating gyro bias...")
        s = 0.0
        for _ in range(samples):
            raw = self.read_word(GYRO_ZOUT_H)
            s += raw / 131.0
            time.sleep(0.002)
        bias = s / samples
        print(f"[IMU] Gyro Z bias: {bias:.3f} dps")
        return bias


class ImuYawNode(Node):
    def __init__(self):
        super().__init__("imu_yaw_node")
        self.pub = self.create_publisher(Float32, "/imu/yaw", 10)

        self.imu = MPU9250Gyro()
        self.last_time = time.time()
        self.yaw = 0.0  # rad

        # 50Hz
        self.timer = self.create_timer(0.02, self.update)
        self.get_logger().info("IMU yaw node started")

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        gz_dps = self.imu.read_gyro_z_dps()
        gz_rad = math.radians(gz_dps)

        self.yaw += gz_rad * dt
        # -pi ~ pi 로 정규화
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi

        msg = Float32()
        msg.data = self.yaw  # rad
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
