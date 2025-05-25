import requests
import cv2
import numpy as np
import time
from flask import Flask, render_template, request, Response, jsonify, abort
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

CAMERA_URL = 'http://192.168.110.110:8000/camera/image'

latest_command = {
    'x': None,
    'y': None,
    'z': None,
    'suction': None
}

latest_joints = [0.0, 0.0, 0.0, 0.0]

homing_state = False
moving_state = False

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


@app.route('/send', methods=['POST'])
def receive_controls():
    global moving_state
    data = request.get_json()
    if not data:
        abort(400)
    
    moving_state = True

    latest_command['x'] = data.get('x', 0)
    latest_command['y'] = data.get('y', 0)
    latest_command['z'] = data.get('z', 0)
    latest_command['suction'] = data.get('suction', False)

    return jsonify({'status': 'ok'}), 200


@app.route('/user_command', methods=['GET'])
def get_latest():
    return jsonify(latest_command), 200

# ros2에서 homing을 완료하면 해당 url로 {"move_done" : true}를 post로 보내야함
@app.route('/move_done', methods=['POST'])
def finish_move():
    global moving_state
    data = request.get_json()

    if not data or 'move_done' not in data:
        abort(400)

    if data.get('move_done') is True:
        moving_state = False
        socketio.emit('move_done', {'status': 'ok'})
        return jsonify({'status': 'ok'}), 200

@app.route('/homing', methods=['POST'])
def start_homing():
    global homing_state
    data = request.get_json()

    if not data or 'homing' not in data:
        abort(400)

    if data['homing'] is True:
        homing_state = True
        return jsonify({'status': 'ok'}), 200
    

@app.route('/homing_state', methods=['GET'])
def get_homing_state():
    return jsonify({'homing': homing_state}), 200

# ros2에서 homing을 완료하면 해당 url로 {"homing_done" : true}를 post로 보내야함
@app.route('/homing_done', methods=['POST'])
def finish_homing():
    global homing_state
    data = request.get_json()
    
    if not data or 'homing_done' not in data:
        abort(400)

    if data.get('homing_done') is True:
        homing_state = False
        socketio.emit('homing_done', {'status': 'ok'})
        return jsonify({'status': 'ok'}), 200
    

@app.route('/update_joints', methods=['POST'])
def update_joints():
    global latest_joints
    data = request.get_json()
    if not data or 'joints' not in data:
        abort(400)

    joints = data['joints']
    corrected = []
    for v in joints:
        try:
            f = float(v)
        except:
            f = 0.0
        if f < 0: f = 0.0
        if f > 100: f = 100.0
        corrected.append(f)

    latest_joints = corrected

    socketio.emit('joints_update', {'joints': latest_joints})
    return jsonify({'status': 'ok'}), 200


@app.route('/image_click', methods=['POST'])
def image_click():
    data = request.get_json()
    if not data or 'x_ratio' not in data or 'y_ratio' not in data:
        abort(400)

    x_ratio = data.get('x_ratio')
    y_ratio = data.get('y_ratio')

    # 비율 값이 숫자인지 확인 (안전책)
    try:
        x_ratio = float(x_ratio)
        y_ratio = float(y_ratio)
    except:
        abort(400)

    # 0.0 ≤ x_ratio ≤ 1.0, 0.0 ≤ y_ratio ≤ 1.0 범위 체크 (선택)
    if not (0.0 <= x_ratio <= 1.0 and 0.0 <= y_ratio <= 1.0):
        abort(400)

    # 예시: 로그로 찍어 보기
    print(f"[ImageClick] x_ratio: {x_ratio:.4f}, y_ratio: {y_ratio:.4f}")

    return jsonify({'status': 'ok'}), 200

# 임시 url (확인용용)
@app.route('/receive_chat', methods=['POST'])
def receive_chat():
    data = request.get_json()
    if not data or 'message' not in data:
        abort(400)

    user_message = data['message']
    print(f"[ChatReceiver] 사용자 메시지: {user_message}")
    return jsonify({'status': 'ok', 'received': user_message}), 200

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=8080)