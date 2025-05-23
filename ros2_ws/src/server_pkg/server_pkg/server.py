import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import time
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from geometry_msgs.msg import Point

def add_offset(pose):
    a_tan = math.atan2(pose[0], pose[1])
    tcp_length = 58.0

    offset_x = math.sin(a_tan)*tcp_length
    offset_y = math.cos(a_tan)*tcp_length
    offset_z = 68.0

    return pose[0] + offset_x, pose[1] + offset_y, pose[2] - offset_z

class DobotServer(Node):
    def __init__(self):
        super().__init__('dobot_status_server')

        self.create_subscription(JointState, '/dobot_joint_states', self.joint_callback, 10)
        self.create_subscription(PoseStamped, '/dobot_TCP', self.tcp_callback, 10)
        self.create_subscription(Bool, '/gripper_status_rviz', self.suction_callback, 10)
        self.create_subscription(Image, '/coord_space_image', self.image_callback, 10)
        self.create_subscription(String, '/detection_results', self.yolo_callback, 10)

        self.publisher_target_point = self.create_publisher(Point, '/target_pos', 10)


    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {math.degrees(pos):.2f} deg')
        print()
        time.sleep(0.05)
        '''
        joint1 : -135 ~ 125 (degree)
        joint2 : -5 ~ 40 (degree)
        joint3 : -15 ~ 80 (degree)
        joint4 : 110 ~ 170 (degree)
        '''

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

        time.sleep(0.05)

    def suction_callback(self, msg):
        self.get_logger().info(f'Suction: {"ON" if msg.data else "OFF"}')
        time.sleep(0.05)

    def image_callback(self, msg):
        
        time.sleep(0.05)

    def yolo_callback(self, msg):
        
        time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = DobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
