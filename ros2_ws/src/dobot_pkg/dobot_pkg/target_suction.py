import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from dobot_msgs.srv import SuctionCupControl

class TargetSuction(Node):
    def __init__(self):
        super().__init__('dobot_target_suction_node')
        self.group = ReentrantCallbackGroup()

        # Suction service client
        self.cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SuctionCupControl.Request()

        # Subscription for suction control
        self.subscription = self.create_subscription(
            Bool,
            'working_suction',
            self.suction_callback,
            10,
            callback_group=self.group
        )

        self.get_logger().info('Suction control node initialized.')

    def suction_callback(self, msg):
        self.get_logger().info(f'Received suction command: {msg.data}')
        self.req.enable_suction = msg.data
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Suction operation successful')
            else:
                self.get_logger().warn('Suction operation failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetSuction()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
