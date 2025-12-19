"""
rotate_mapper.py
- rotate_mapper_node.py 의 ROS2 의존성을 제거한 순수 Python 버전
- Supervisor가:
    mapper.start()
    ...
    mapper.stop()
  이렇게 호출함.
"""

import time
import json
import math
import statistics
import threading
import os


class RotateMapper:
    def __init__(self, tof_reader, imu_reader,
                 save_path="/home/pi/Desktop/PPOR_BALL/scan_current.json",
                 interval=0.05):
        """
        tof_reader: ToFReader 인스턴스
        imu_reader: ImuReader 인스턴스
        interval  : 샘플링 주기(초)
        """
        self.tof_reader = tof_reader
        self.imu_reader = imu_reader
        self.save_path = save_path
        self.interval = interval

        self.mapping_active = False

        #self.sensor_names = ["front", "left", "right", "back"]
        self.sensor_names = ["front", "left", "back"]
        self.samples = {k: [] for k in self.sensor_names}

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ------------------------------------------
    # 외부에서 호출: rotate start
    # ------------------------------------------
    def start(self):
        self.samples = {k: [] for k in self.sensor_names}
        self.mapping_active = True
        print("[RotateMapper] START scan")

    # ------------------------------------------
    # 외부에서 호출: rotate stop → scan 저장
    # ------------------------------------------
    def stop(self):
        self.mapping_active = False
        print("[RotateMapper] STOP scan → saving…")
        self._save_scan()

    # ------------------------------------------
    # 내부 루프: mapping_active면 ToF 샘플 기록
    # ------------------------------------------
    def _loop(self):
        while True:
            if self.mapping_active:
                tof = self.tof_reader.get_distances()
                yaw = self.imu_reader.get_yaw()

                for name in self.sensor_names:
                    d = tof.get(name)
                    if d is not None and d > 0.02:
                        self.samples[name].append(d)

            time.sleep(self.interval)

    # ------------------------------------------
    # median 기반 scan 저장
    # ------------------------------------------
    def _save_scan(self):
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
            #"right": summary.get("right"),
            "back": summary.get("back"),
            "raw": raw,
        }

        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, "w") as f:
                json.dump(data, f, indent=2)

            print(f"[RotateMapper] Saved scan -> {self.save_path}")
        except Exception as e:
            print(f"[RotateMapper] Save failed: {e}")

    # Supervisor가 스캔 데이터를 즉시 사용하고 싶을 경우:
    def get_last_scan(self):
        return dict(self.samples)