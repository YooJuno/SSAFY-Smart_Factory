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
import os

dobot_status = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, 'templates'),
    static_folder=os.path.join(BASE_DIR, 'static')
)
ROS_DELAY_INTERVAL = 0.1
server_node = None

TCP_HOST = '192.168.110.110'  # 서버 IP 주소
TCP_PORT = 40000              # 포트 번호

socketio = SocketIO(app, cors_allowed_origins="*")
CORS(app, resources={r"/*": {"origins": "*"}})

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
        dobot_status[3] = math.degrees(msg.position[0])
        dobot_status[4] = math.degrees(msg.position[1])
        dobot_status[5] = math.degrees(msg.position[2])
        dobot_status[6] = math.degrees(msg.position[3])

        # self.get_logger().info(
        #     f"Joint1: {dobot_status[3]:.2f} deg, "
        #     f"Joint2: {dobot_status[4]:.2f} deg, "
        #     f"Joint3: {dobot_status[5]:.2f} deg, "
        #     f"Joint4: {dobot_status[6]:.2f} deg"
        # )

        '''
        joint1 : -135 ~ 125 (degree)
        joint2 : -5 ~ 40 (degree)
        joint3 : -15 ~ 80 (degree)
        joint4 : 110 ~ 170 (degree)
        '''
        time.sleep(ROS_DELAY_INTERVAL)

    # 완료
    def tcp_callback(self, msg):
        pose = (msg.pose.position.x*1000, msg.pose.position.y*1000, msg.pose.position.z*1000)
        pose = add_offset(pose)

        # self.get_logger().info(
        #     f'TCP Pose: x={(pose[0]):.2f}, y={pose[1]:.2f}, z={pose[2]:.2f}'
        # )
        dobot_status[0] = pose[0] # x
        dobot_status[1] = pose[1] # y
        dobot_status[2] = pose[2] # z
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


# 프레임을 웹에 띄울 수 있게 인코딩
def generate_stream():
    while True:
        image = server_node.get_jpeg_image()
        if image is None:
            # 아직 한 프레임도 안 들어왔으면 잠시 대기 후 loop 재시작
            time.sleep(0.03)
            continue

        # 이 시점에서는 image가 bytes 타입(JPEG 데이터)이므로 안전하게 처리
        try:
            image_array = np.frombuffer(image, np.uint8)
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        except Exception as e:
            print(f"[generate_stream] 디코딩 에러: {e}")
            time.sleep(0.03)
            continue

        # frame이 유효하면 JPEG으로 다시 인코딩해서 yield
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            # 인코딩 실패 시에도 잠시 대기
            time.sleep(0.03)
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        # time.sleep(1.0 / 30.0)

# html 렌더링 (html, css, js 모두 렌더링됨)
@app.route('/')
def index():
    return render_template('index.html')

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

    '''
    dobot 움직임 명령
    '''
    print(f"[Receiver] 사용자 메시지: {latest_command[0], latest_command[1], latest_command[2], latest_command[3]}")
    return jsonify(latest_command), 200


# 사용자가 Homing 버튼을 눌렀다는 것을 app.js를 통해 받아옴 
@app.route('/homing', methods=['POST'])
def start_homing():
    '''
    유준호밍
    '''
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

def broadcast_status():
    # ROS 노드가 준비될 때까지 잠시 대기
    while server_node is None:
        time.sleep(0.05)

    # joint 각도를 0~100 비율로 변환할 때 쓸 범위(예제)
    joint_ranges = [
        (-135.0, 125.0),  # joint1
        (-5.0, 40.0),     # joint2
        (-15.0, 80.0),    # joint3
        (110.0, 170.0)    # joint4
    ]

    while True:
        # ① dobot_status[0:3] → x_mm, y_mm, z_mm
        x_mm = dobot_status[0]
        y_mm = dobot_status[1]
        z_mm = dobot_status[2]

        # ② dobot_status[3:7] (deg) → 0~100 비율
        percents = []
        for i in range(4):
            deg = dobot_status[3 + i]
            lo, hi = joint_ranges[i]
            p = (deg - lo) / (hi - lo) * 100.0
            p = max(0.0, min(100.0, p))
            percents.append(p)

        # ③ dobot_status[7] → Boolean
        suction_bool = (dobot_status[7] >= 0.5)

        # ④ Socket.IO로 emit
        socketio.emit('state_update', {
            'x': x_mm,
            'y': y_mm,
            'z': z_mm,
            'joints': percents,
            'suction': suction_bool
        })

        # 1초 대기
        time.sleep(0.5)


@socketio.on('connect')
def on_connect():
    # 클라이언트가 최초 연결될 때 broadcast_status를 background task로 실행
    # 한 번만 실행되도록, 이미 실행 중인지 체크할 수도 있음.
    # 단순히 여러 클라이언트가 연결돼도 한 번만 실행되도록 flag를 쓰는 예:
    if not hasattr(on_connect, 'started'):
        on_connect.started = True
        socketio.start_background_task(broadcast_status)


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

    socketio.run(app, host='0.0.0.0', port=65433, allow_unsafe_werkzeug=True, debug=True, use_reloader=False)

if __name__ == '__main__':
    main()
