import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from geometry_msgs.msg import Point

from cv_bridge import CvBridge
import threading, time, math, cv2, socket
import numpy as np

from flask_cors import CORS
from flask_socketio import SocketIO
from flask import Flask, render_template, request, Response, jsonify, abort
import requests

app = Flask(__name__)

ROS_DELAY_INTERVAL = 0.1
server_node = None

TCP_HOST = '192.168.110.110'  # 서버 IP 주소
TCP_PORT = 40000              # 포트 번호

socketio = SocketIO(app, cors_allowed_origins="*")
CORS(app, resources={r"/*": {"origins":"*"}})

latest_command = {
    'x': None,
    'y': None,
    'z': None,
    'suction': None
}

latest_joints = [0.0, 0.0, 0.0, 0.0]

homing_state = False
moving_state = False

def add_offset(pose):
    a_tan = math.atan2(pose[0], pose[1])
    tcp_length = 58.0

    offset_x = math.sin(a_tan)*tcp_length
    offset_y = math.cos(a_tan)*tcp_length
    offset_z = 68.0

    return pose[0] + offset_x, pose[1] + offset_y, pose[2] - offset_z

class DobotServerNode(Node):
    def __init__(self):
        super().__init__('server_node')

        self.create_subscription(JointState, '/dobot_joint_states', self.joint_callback, 10)
        self.create_subscription(PoseStamped, '/dobot_TCP', self.tcp_callback, 10)
        self.create_subscription(Bool, '/gripper_status_rviz', self.suction_callback, 10)
        self.create_subscription(Image, '/coord_space_image', self.coord_board_image_callback, 10)
        self.create_subscription(String, '/detection_results', self.yolo_callback, 10)

        self.publisher_target_point = self.create_publisher(Point, '/target_pos', 10)

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.latest_frame = None
        
    def joint_callback(self, msg):
        # for name, pos in zip(msg.name, msg.position):
        #     self.get_logger().info(f'{name}: {math.degrees(pos):.2f} deg')
        # print()
        '''
        joint1 : -135 ~ 125 (degree)
        joint2 : -5 ~ 40 (degree)
        joint3 : -15 ~ 80 (degree)
        joint4 : 110 ~ 170 (degree)
        '''
        time.sleep(ROS_DELAY_INTERVAL)

    def tcp_callback(self, msg):
        pose = (msg.pose.position.x*1000, msg.pose.position.y*1000, msg.pose.position.z*1000)
        pose = add_offset(pose)

        # self.get_logger().info(
        #     f'TCP Pose: x={(pose[0]):.2f}, y={pose[1]:.2f}, z={pose[2]:.2f}'
        # )
        '''
        Y : -100 ~ 320 (mm)
        Y : -280 ~ 300 (mm)
        Z : -130 ~ 113 (mm)
        '''

        time.sleep(ROS_DELAY_INTERVAL)

    def suction_callback(self, msg):
        self.get_logger().info(f'Suction: {"ON" if msg.data else "OFF"}')
        time.sleep(ROS_DELAY_INTERVAL)

    def coord_board_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (cv_image.shape[1]//2, cv_image.shape[0]//2))
            with self.lock:
                self.latest_frame = cv_image.copy()
            # cv2.imshow("RealSense Color Stream", cv_image)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"CV conversion failed: {e}")
        time.sleep(ROS_DELAY_INTERVAL)

    def yolo_callback(self, msg):
        
        time.sleep(ROS_DELAY_INTERVAL)

    def get_jpeg_image(self):
        global server_node
        with self.lock:
            if self.latest_frame is None:
                return None
            success, jpeg = cv2.imencode('.jpg', self.latest_frame)
            if success:
                return jpeg.tobytes()
            return None


# 이미지 서버에서 프레임 가져오기
def get_frame():
    image = server_node.get_jpeg_image()
    image_array = np.frombuffer(image, np.uint8)
    frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    return frame

# 프레임을 웹에 띄울 수 있게 인코딩
def generate_stream():
    while True:
        frame = get_frame()
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        # time.sleep(1.0 / 30.0)

# html 렌더링 (html, css, js 모두 렌더링됨)
# @app.route('/')
# def index():
#     return render_template('index.html')

# 웹페이지에 받아온 프레임을 동영상으로 띄워줌
@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# 사용자가 x, y, z, suction_cup 값 지정 후 send 버튼 누름 -> app.js에서 값을 받아옴
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

    return jsonify(latest_command), 200

# /send에서 받아온 데이터 확인용(json 형식)
@app.route('/user_command', methods=['GET'])
def get_latest():
    return jsonify(latest_command), 200

# 두봇의 움직임이 다 끝났을 때 post 요청 -> 웹페이지에서 모달 사라짐(사용자 조작 가능해짐)
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


# 사용자가 Homing 버튼을 눌렀다는 것을 app.js를 통해 받아옴 
@app.route('/homing', methods=['POST'])
def start_homing():
    global homing_state
    data = request.get_json()

    if not data or 'homing' not in data:
        abort(400)

    if data['homing'] is True:
        homing_state = True
        return jsonify({'status': 'ok'}), 200
    
# json 형식으로 {"homing":true} 반환
@app.route('/homing_state', methods=['GET'])
def get_homing_state():
    return jsonify({'homing': homing_state}), 200

# 두봇의 움직임이 다 끝났을 때 post 요청 -> 웹페이지에서 모달 사라짐(사용자 조작 가능해짐)
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
    

# 해당 url로 {"joints":[f1, f2, f3, f4]} 값을 보내면 도넛 차트 업데이트
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


# 동영상에서 사용자가 클릭한곳을 비율로 받아옴
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

# 사용자가 chat gpt에 입력한 문장을 받아옴
@app.route('/receive_chat', methods=['POST'])
def receive_chat():
    data = request.get_json()
    if not data or 'message' not in data:
        abort(400)

    user_message = data['message']
    print(f"[ChatReceiver] 사용자 메시지: {user_message}")
    return jsonify({'status': 'ok', 'received': user_message}), 200


def ros_thread_main():
    global server_node
    rclpy.init()
    server_node = DobotServerNode()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()


def tcp_server_main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_HOST, TCP_PORT))
    server_socket.listen(1)
    print(f"[TCP] 서버가 {TCP_HOST}:{TCP_PORT}에서 대기 중입니다...")

    client_socket, addr = server_socket.accept()
    print(f"[TCP] 클라이언트 연결됨: {addr}")

    try:
        while True:
            msg = input("클라이언트로 보낼 색상(red, white, blue 등) 입력 (종료: exit): ").strip()
            if msg.lower() == "exit":
                print("[TCP] 서버를 종료합니다.")
                break
            if not msg:
                continue  # 빈 입력 무시

            client_socket.sendall(msg.encode('utf-8'))
            print(f"[TCP] 전송 완료: {msg}")

    except Exception as e:
        print(f"[TCP] 에러 발생: {e}")

    finally:
        client_socket.close()
        server_socket.close()
        print("[TCP] 서버 소켓 종료.")


def main():
    ros_thread = threading.Thread(target=ros_thread_main, daemon=True)
    ros_thread.start()

    # tcp_thread = threading.Thread(target=tcp_server_main, daemon=True)
    # tcp_thread.start()

    # app.run(host='0.0.0.0', port=8000)
    # socketio.run(app, host='0.0.0.0', port=8080)
    socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True, debug=True, use_reloader=False)





if __name__ == '__main__':
    main()
