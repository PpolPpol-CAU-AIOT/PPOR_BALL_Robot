# stream_server.py
# A simple MJPEG streaming server using Flask and Picamera2 (recommended for modern Raspberry Pi OS)
# Run this script on your Raspberry Pi, then access http://<pi-ip-address>:9000/ for a simple viewer page,
# or directly view the stream at http://<pi-ip-address>:9000/stream

from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from threading import Condition
import io
import logging
import libcamera
logging.getLogger().setLevel(logging.INFO)

app = Flask(__name__)

# Streaming output class to buffer frames
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

# Initialize camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (640, 480)},
    transform=libcamera.Transform(hflip=1,vflip=1)
)
picam2.configure(video_config)

output = StreamingOutput()
encoder = MJPEGEncoder(bitrate=5000000)  # Adjust bitrate for quality/framerate
picam2.start_encoder(encoder, FileOutput(output))
picam2.start()

# Generator for MJPEG stream
def gen_stream():
    while True:
        with output.condition:
            output.condition.wait()
            frame = output.frame
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/stream')
def stream():
    return Response(gen_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    # Simple HTML page to view the stream
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Raspberry Pi MJPEG Stream</title>
    </head>
    <body>
        <h1>Raspberry Pi Camera MJPEG Stream</h1>
        <img src="{{ url_for('stream') }}" width="640" height="480" />
    </body>
    </html>
    """)

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=9000, threaded=True)
    finally:
        picam2.stop_encoder()
        picam2.stop()
