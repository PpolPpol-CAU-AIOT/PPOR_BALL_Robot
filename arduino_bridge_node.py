#!/usr/bin/env python3
import serial
import threading

class ArduinoBridge:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.01):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lock = threading.Lock()

        try:
            print(f"[Arduino] Opening {port} ...")
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.connected = True
            print("[Arduino] Connected.")
        except Exception as e:
            print(f"[Arduino] Serial open failed: {e}")
            self.ser = None
            self.connected = False


    def send(self, r: int, l: int, h: int):
        """r, l, h는 -100 ~ 100 정수"""
        if not self.connected:
            return

        # 클램프
        r = max(min(int(r), 100), -100)
        l = max(min(int(l), 100), -100)
        h = max(min(int(h), 100), -100)

        cmd = f"r{r},l{l},h{h}\ns"

        with self.lock:
            try:
                self.ser.write(cmd.encode())
                #print(f"[Arduino] send move commendssss {cmd}")  # 필요시 디버깅
            except Exception as e:
                print(f"[Arduino] Write error: {e}")
                self.connected = False

    def stop(self):
        self.send(0, 0, 0)

    def close(self):
        self.stop()
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.connected = False
