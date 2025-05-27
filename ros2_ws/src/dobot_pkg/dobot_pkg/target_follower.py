import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

def add_offset(x, y, z):
    a_tan = math.atan2(x, y)
    tcp_length = 58

    offset_x = math.sin(a_tan) * tcp_length
    offset_y = math.cos(a_tan) * tcp_length
    offset_z = 68

    return x - offset_x, y - offset_y, z + offset_z

class TargetFollower(Node):
    def __init__(self):
        super().__init__('dobot_target_follower_node')
        self.group = ReentrantCallbackGroup()

        # Action client for movement
        self.action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=self.group)
        
        # Service client for suction
        self.suction_cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        while not self.suction_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Suction service not available, waiting...')
        self.suction_req = SuctionCupControl.Request()

        # Subscriptions
        self.target_sub = self.create_subscription(
            Point,
            '/target_pos',
            self.target_callback,
            10,
            callback_group=self.group
        )
        
        self.suction_sub = self.create_subscription(
            Bool,
            '/working_suction',
            self.suction_callback,
            10,
            callback_group=self.group
        )

        self.busy = False
        self.get_logger().info('Dobot controller node initialized.')

    #region Movement Control
    def target_callback(self, msg):
        if self.busy:
            self.get_logger().info('Dobot busy - ignoring movement command')
            return

        self.get_logger().info(f'New target received: X:{msg.x:.2f}, Y:{msg.y:.2f}, Z:{msg.z:.2f}')
        result_x, result_y, result_z = add_offset(msg.x, msg.y, msg.z)

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = [result_x, result_y, result_z, 0.0]
        goal_msg.motion_type = 1

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
            self.get_logger().info('Movement goal rejected')
            self.busy = False
            return

        self.get_logger().info('Movement in progress...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Movement completed successfully')
        else:
            self.get_logger().warn(f'Movement failed with status: {status}')
        self.busy = False

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'Movement progress: {feedback_msg.feedback}')
    #endregion

    #region Suction Control
    def suction_callback(self, msg):
        self.get_logger().info(f'Received suction command: {msg.data}')
        self.suction_req.enable_suction = msg.data
        future = self.suction_cli.call_async(self.suction_req)
        future.add_done_callback(self.suction_response_callback)

    def suction_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Suction state updated successfully')
            else:
                self.get_logger().warn('Suction operation failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    #endregion

def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
