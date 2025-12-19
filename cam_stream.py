from flask import Flask, Response
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import io

app = Flask(__name__)

# Initialize camera only once
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
time.sleep(2)  # allow camera to warm up

def mjpeg_stream():
    stream = io.BytesIO()
    for _ in camera.capture_continuous(stream, format="jpeg", use_video_port=True):
        stream.seek(0)
        frame = stream.read()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

        stream.seek(0)
        stream.truncate()

@app.route("/stream")
def stream():
    return Response(
        mjpeg_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=9000, threaded=True)

