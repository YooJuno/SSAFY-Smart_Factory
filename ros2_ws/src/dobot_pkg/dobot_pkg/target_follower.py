import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

from dobot_msgs.action import PointToPoint

from dobot_msgs.srv import SuctionCupControl

import math

def add_offset(x, y, z):
    a_tan = math.atan2(x, y)
    tcp_length = 58

    offset_x = math.sin(a_tan)*tcp_length
    offset_y = math.cos(a_tan)*tcp_length
    offset_z = 68

    return x - offset_x, y - offset_y, z + offset_z

class TargetFollower(Node):
    def __init__(self):
        super().__init__('dobot_target_follower_node')

        self.group = ReentrantCallbackGroup()

        # 액션 클라이언트
        self.action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=self.group)
        self.cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SuctionCupControl.Request()


        # 토픽 구독
        self.subscription = self.create_subscription(
            Point,
            '/target_pos',
            self.target_callback,
            10,
            callback_group=self.group
        )

        self.busy = False  # 이동 중인지 확인
        self.get_logger().info('Target follower node initialized.')

    def target_callback(self, msg):
        if self.busy:
            self.get_logger().info('Dobot is busy. Ignoring target.')
            return

        self.get_logger().info(f'Received target: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

        goal_msg = PointToPoint.Goal()
        result_x, result_y, result_z = add_offset(msg.x, msg.y, 0.0)

        goal_msg.target_pose = [result_y, result_x, 0.0, 0.0]  # Z, R은 필요시 조정
        goal_msg.motion_type = 1  # 1 = MOVEL (선형 이동), 0 = MOVEJ (조인트 이동)

        self.action_client.wait_for_server()
        self.busy = True
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            self.busy = False
            return

        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Movement succeeded.')
        else:
            self.get_logger().warn('Movement failed or was canceled.')
        self.busy = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
