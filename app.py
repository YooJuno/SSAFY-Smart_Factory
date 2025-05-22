import requests
import cv2
import numpy as np
import time
from flask import Flask, render_template, request, Response, redirect, url_for, send_file

app = Flask(__name__)

CAMERA_URL = 'http://192.168.110.110:8000/camera/image'

def get_frame():
    try:
        response = requests.get(CAMERA_URL, timeout=1.0)
        if response.status_code == 200:
            image_array = np.frombuffer(response.content, np.uint8)
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            return frame
    except Exception as e:
        print("Error getting frame:", e)
    return None

def generate_stream():
    while True:
        frame = get_frame()
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(1.0 / 30.0)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host = '0.0.0.0', port=8080)