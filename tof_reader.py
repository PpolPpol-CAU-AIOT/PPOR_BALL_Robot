"""
tof_reader.py
- sensor_manager.ToFSensorManager 를 스레드로 감싸서
  주기적으로 읽은 값을 shared dict에 저장한다.
- ROS 사용 X
"""

import threading
import time
from sensor_manager import ToFSensorManager


class ToFReader:
    def __init__(self, interval=0.05):
        self.interval = interval

        gpio_map = {
            "front": 5,
            "left": 16,
            #"right": 6,
            "back": 26,
        }

        address_map = {
            "front": 0x30,
            "left": 0x31,
            #"right": 0x32,
            "back": 0x33,
        }

        # VL53L0X 초기화
        self.manager = ToFSensorManager(gpio_map, address_map)

        self.data = {
            "front": None,
            "left": None,
            #"right": None,
            "back": None,
        }

        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # --------------------------
    # 내부 루프: 20Hz (기본 50ms)
    # --------------------------
    def _loop(self):
        while self.running:
            new_data = self.manager.read_all()    # {front:1.23, left:...}

            with self.lock:
                self.data.update(new_data)

            time.sleep(self.interval)

    def get_distances(self):
        """현재 측정값 dict 반환"""
        with self.lock:
            return dict(self.data)

    def stop(self):
        self.running = False
        self.thread.join()
