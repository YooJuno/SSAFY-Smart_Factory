import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState, Image 

from cv_bridge import CvBridge
import threading, time, math, cv2, socket, os
import numpy as np

from flask_cors import CORS
from flask import Flask, render_template, request, Response, jsonify, abort
import requests

dobot_status = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, 'templates'),
    static_folder=os.path.join(BASE_DIR, 'static')
)
ROS_DELAY_INTERVAL = 0.1
server_node = None
is_suction_working = False

TCP_HOST = '192.168.110.110'  # 서버 IP 주소
TCP_PORT = 40000              # 포트 번호

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
        self.publisher_homing = self.create_publisher(String, '/go_home', 10)
        self.publisher_chat = self.create_publisher(String, '/chat_input', 10)
        self.publisher_point = self.create_publisher(Point, '/target_pos', 10)
        self.publisher_suction = self.create_publisher(Bool, '/working_suction', 10)

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.latest_frame = None
        
    '''
    joint1 : -135 ~ 125 (degree)
    joint2 : -5 ~ 40 (degree)
    joint3 : -15 ~ 80 (degree)
    joint4 : 110 ~ 170 (degree)
    '''
    def joint_callback(self, msg):
        dobot_status[3] = math.degrees(msg.position[0])
        dobot_status[4] = math.degrees(msg.position[1])
        dobot_status[5] = math.degrees(msg.position[2])
        dobot_status[6] = math.degrees(msg.position[3])

        time.sleep(ROS_DELAY_INTERVAL)

    '''
    Y : -100 ~ 320 (mm)
    Y : -280 ~ 300 (mm)
    Z : -130 ~ 113 (mm)
    '''
    def tcp_callback(self, msg):
        pose = (msg.pose.position.x*1000, msg.pose.position.y*1000, msg.pose.position.z*1000)
        pose = add_offset(pose)

        dobot_status[0] = pose[0] # x
        dobot_status[1] = pose[1] # y
        dobot_status[2] = pose[2] # z

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

@app.route('/camera/coordinate_board')
def get_image():
    global server_node
    if server_node is None:
        return Response("ROS2 node not initialized", status=500)

    image = server_node.get_jpeg_image()
    if image:
        return Response(response=image, content_type='image/jpeg')
    else:
        return Response("No image available", status=404)


                    
# html 렌더링 (html, css, js 모두 렌더링됨)
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/status', methods=['GET'])
def status():
    status = {
        'x': dobot_status[0],
        'y': dobot_status[1],
        'z': dobot_status[2],
        'joints1': dobot_status[3],
        'joints2': dobot_status[4],
        'joints3': dobot_status[5],
        'joints4': dobot_status[6],
        'suction': is_suction_working
    }
    return jsonify(status), 200


# 사용자가 x, y, z, suction_cup 값 지정 후 send 버튼 누름 -> app.js에서 값을 받아옴
@app.route('/send', methods=['POST'])
def receive_controls():
    global server_node, is_suction_working
    data = request.get_json()

    point = Point()
    point.x = float(data.get('x', 0))
    point.y = float(data.get('y', 0))
    point.z = float(data.get('z', 0))

    suction_working = Bool()
    suction_working.data = data.get('suction')
    is_suction_working = suction_working.data

    server_node.publisher_point.publish(point)
    server_node.publisher_suction.publish(suction_working)

    return jsonify({'status': 'ok'}), 200


@app.route('/homing', methods=['POST'])
def start_homing():
    global server_node
    from std_msgs.msg import String
    msg = String()
    msg.data = 'go_home'
    server_node.publisher_homing.publish(msg)
    return jsonify({'status': 'ok'}), 200
    

# 사용자가 chat gpt에 입력한 문장을 받아옴
@app.route('/receive_chat', methods=['POST'])
def receive_chat():
    global server_node
    data = request.get_json()

    msg = String()
    user_message = str(data['message'])
    msg.data = user_message
    server_node.publisher_chat.publish(msg)
    server_node.get_logger().info(f'{msg.data}')

    return jsonify({'status': 'ok'}), 200


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

    app.run(host='0.0.0.0', port=65432, debug=True, use_reloader=False)

if __name__ == '__main__':
    main()