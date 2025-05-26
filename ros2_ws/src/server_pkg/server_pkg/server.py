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

from flask import Flask, Response


app = Flask(__name__)

ROS_DELAY_INTERVAL = 0
server_node = None

TCP_HOST = '192.168.110.110'  # 서버 IP 주소
TCP_PORT = 40000              # 포트 번호

def add_offset(pose):
    a_tan = math.atan2(pose[0], pose[1])
    tcp_length = 58.0

    offset_x = math.sin(a_tan)*tcp_length
    offset_y = math.cos(a_tan)*tcp_length
    offset_z = 68.0

    return pose[0] + offset_x, pose[1] + offset_y, pose[2] - offset_z

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
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {math.degrees(pos):.2f} deg')
        print()
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

        self.get_logger().info(
            f'TCP Pose: x={(pose[0]):.2f}, y={pose[1]:.2f}, z={pose[2]:.2f}'
        )
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
            cv2.imshow("RealSense Color Stream", cv_image)
            cv2.waitKey(1)
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


@app.route('/camera/image')
def get_image():
    image = server_node.get_jpeg_image()
    if image:
        return Response(response=image, content_type='image/jpeg')
    else:
        return Response("No image available", status=404)


def ros_thread_main():
    global server_node
    rclpy.init()
    server_node = DobotServerNode()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

def main():
    ros_thread = threading.Thread(target=ros_thread_main, daemon=True)
    ros_thread.start()

    tcp_thread = threading.Thread(target=tcp_server_main, daemon=True)
    tcp_thread.start()

    app.run(host='0.0.0.0', port=8000)


if __name__ == '__main__':
    main()