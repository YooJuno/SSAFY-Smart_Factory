import rclpy
from rclpy.node import Node
from dobot_msgs.srv import ExecuteHomingProcedure
from std_msgs.msg import String
import sys

class Node1(Node):
    def __init__(self):
        super().__init__('dobot_homing_service_node')
        self.get_logger().info('dobot_homing_service has started.')

        self.cli = self.create_client(ExecuteHomingProcedure, 'dobot_homing_service')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request()

        self.subscription = self.create_subscription(
            String,
            '/go_home',
            self.homing,
            10
        )

    def send_request(self):
        request = ExecuteHomingProcedure.Request()

        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def homing(self, msg):
        self.send_request()

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Homing Procedure executed successfully: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Node1()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()