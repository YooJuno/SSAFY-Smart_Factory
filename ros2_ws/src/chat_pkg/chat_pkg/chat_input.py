import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('chat_input_publisher')
        self.publisher_ = self.create_publisher(String, '/chat_input', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        user_input = input('input: ')
        msg = String()
        msg.data = user_input
        self.publisher_.publish(msg)
        self.get_logger().info(f'{user_input}')

def main(args=None):
    rclpy.init(args=args)
    node = UserInputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
