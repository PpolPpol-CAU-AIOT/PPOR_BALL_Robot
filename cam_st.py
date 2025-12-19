#!/usr/bin/env python3
import cv2
import threading
import time
from flask import Flask, Response
import signal
import sys

app = Flask(__name__)

CAM_DEVICE = -1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 15

class Camera:
    def __init__(self, device=0, width=640, height=480, fps=15):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self._open_camera() 

        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _open_camera(self):
        """Open or reopen the camera safely"""
        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            print("[Camera] Failed to open camera")
            time.sleep(1)
            return self._open_camera()

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        print("[Camera] Camera opened")

    def _reader(self):
        """Continuously read frames with FPS throttling"""
        frame_interval = 1.0 / self.fps
        last_time = 0

        while self.running:
            now = time.time()
            if now - last_time < frame_interval:
                time.sleep(0.001)
                continue
            last_time = now

            ret, frame = self.cap.read()
            if not ret:
                print("[Camera] Read failed. Reopening...")
                self._open_camera()
                continue

            with self.lock:
                self.frame = frame

    def get_frame(self):
        """Return JPEG encoded frame"""
        with self.lock:
            if self.frame is None:
                return None
            ret, jpeg = cv2.imencode('.jpg', self.frame)
            return jpeg.tobytes()

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if self.cap:
            self.cap.release()
        print("[Camera] Stopped")


camera = Camera(CAM_DEVICE, FRAME_WIDTH, FRAME_HEIGHT, FPS)

@app.route("/stream")
def stream():
    def generate():
        while True:
            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

def shutdown():
    print("\n[Server] Shutting down...")
    camera.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, lambda a,b: shutdown())

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=9000, threaded=True)
