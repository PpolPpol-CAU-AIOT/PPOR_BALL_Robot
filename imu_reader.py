import smbus2
import time
import math
import threading

MPU_ADDR = 0x68

# 레지스터
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47


class MPU9250Gyro:
    def __init__(self, bus_id=1, addr=MPU_ADDR):
        self.bus = smbus2.SMBus(bus_id)
        self.addr = addr

        # IMU 깨우기
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Gyro full-scale = ±250 dps
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        time.sleep(0.1)

        # bias 측정
        self.bias_z = self._calibrate_bias()

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) | low
        # signed
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read_gyro_z_dps(self):
        raw = self._read_word(GYRO_ZOUT_H)
        dps = raw / 131.0  # sensitivity
        dps -= self.bias_z

        # 노이즈 컷
        if abs(dps) < 0.1:
            dps = 0.0

        return dps

    def _calibrate_bias(self, samples=500):
        print("[IMU] Calibrating gyro Z bias...")
        s = 0.0
        for _ in range(samples):
            raw = self._read_word(GYRO_ZOUT_H)
            s += raw / 131.0
            time.sleep(0.002)
        bias = s / samples
        print(f"[IMU] Bias Z = {bias:.3f} dps")
        return bias


class ImuReader:
    """
    50Hz 루프로 yaw 적분, -pi ~ pi로 유지.
    supervisor는 imu.get_yaw() 를 계속 불러서 쓰면 된다.
    """
    def __init__(self):
        self.imu = MPU9250Gyro()
        self.yaw = 0.0  # rad
        self.lock = threading.Lock()

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        last = time.time()
        while self.running:
            now = time.time()
            dt = now - last
            last = now

            gz_dps = self.imu.read_gyro_z_dps()
            gz_rad = math.radians(gz_dps)

            # yaw 적분
            with self.lock:
                self.yaw += gz_rad * dt
                # -pi ~ pi wrap
                self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

            time.sleep(0.02)  # 50Hz

    def get_yaw(self):
        with self.lock:
            return self.yaw

    def stop(self):
        self.running = False
        self.thread.join()