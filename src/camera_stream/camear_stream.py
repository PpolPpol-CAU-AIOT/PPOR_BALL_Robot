from flask import Flask, Response, render_template_string, request
import cv2
import threading
import time
import signal
import sys

app = Flask(__name__)

CAM_DEVICE = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 15

class Camera:
    def __init__(self, device=0, width=640, height=480, fps=15):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.lock = threading.Lock()
        self.running = True
        self.frame = None

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()