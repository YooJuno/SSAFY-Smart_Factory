import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
import time
import json

from openai import OpenAI
client = OpenAI(
    api_key="sk-proj-GsbdPOxiDTDiFwopcpstT3BlbkFJKeqJCD48skZSLAeTKn16"
)

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher_node')
        self.publisher_pos = self.create_publisher(Point, '/target_pos', 10)
        self.publisher_suction = self.create_publisher(Bool, '/working_suction', 10)
        self.subscription = self.create_subscription(
            String,
            '/chat_input',
            self.user_input_callback,
            10
        )
        self.get_logger().info('/chat_input 토픽에서 입력을 기다립니다.')

    def user_input_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f'입력 수신: "{user_input}"')
        try:
            responses = self.get_gpt_response(user_input)
        except Exception as e:
            self.get_logger().error(f'GPT 응답 오류: {e}')
            return
        
        for response in responses:
            if response["cmd"] == "move":
                try:
                    x = response["xyz"]['x']
                    y = response["xyz"]['y']
                    z = response["xyz"]['z']
                except Exception as e:
                    self.get_logger().error('좌표 파싱 오류')
                    continue

                msg = Point()
                msg.x = float(x)
                msg.y = float(y)
                msg.z = float(z)
                self.publisher_pos.publish(msg)
                self.get_logger().info(f'좌표 전송 완료: ({x}, {y}, {z})')
            else:
                try:
                    is_suction = response["is_suction"]
                    is_suction = bool(is_suction)
                except Exception as e:
                    self.get_logger().error('논리값 파싱 오류')
                    continue
                msg = Bool()
                msg.data = is_suction
                self.publisher_suction.publish(msg)
                self.get_logger().info(f'suction 전송 완료: {is_suction}')

            time.sleep(2)
    
    def get_gpt_response(self, user_input):
        system_message = '''
            You should use your input to convert the user's input into a command by responding in JSON format.
            Analyze the user's input and respond in JSON format.

            example)
            [
                {
                    "cmd": "move",
                    "xyz": {
                                "x": {userinput},
                                "y": {userinput},
                                "z": {userinput}
                            }
                },
                {
                    "cmd": "suction",
                    "is_suction": 1
                },
                {
                    "cmd": "move",
                    "xyz": {
                                "x": {userinput},
                                "y": {userinput},
                                "z": {userinput}
                            }
                },
                {
                    "cmd": "suction",
                    "is_suction": 0
                }, ...
            ]
            
            You must not add anything other than JSON format (such as comments) to your response so that it can be parsed by a JSON parser in Python.
        '''
        messages = [
            {
                "role": "system",
                "content": system_message
            },
            {
                "role": "user",
                "content": user_input
            }
        ]
        response = client.responses.create(
            model="gpt-4.1-nano",
            input=messages
        )

        print(response.output_text)

        return json.loads(response.output_text)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
