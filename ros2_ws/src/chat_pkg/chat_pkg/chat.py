import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time
import json

from openai import OpenAI


import os
from dotenv import load_dotenv

client = OpenAI(
    api_key = 'sk-proj-GsbdPOxiDTDiFwopcpstT3BlbkFJKeqJCD48skZSLAeTKn16'
)

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('chat_node')
        self.publisher_ = self.create_publisher(Point, '/target_pos', 10)
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
            try:
                x = response['x']
                y = response['y']
                z = response['z']
            except Exception as e:
                self.get_logger().error('좌표 파싱 오류')
                continue

            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            self.publisher_.publish(msg)
            self.get_logger().info(f'좌표 전송 완료: ({x}, {y}, {z})')
            time.sleep(2)
    
    def get_gpt_response(self, user_input):
        system_message = '''
            You should use your input to convert the user's input into a command by responding in JSON format.
            Analyze the user's input and respond in JSON format.

            example)
            [{
                "x": {userinput},
                "y": {userinput},
                "z": {userinput}
            },
            {
                "x": {userinput},
                "y": {userinput},
                "z": {userinput}
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
            # model="gpt-4.1-nano",
            model='gpt-4o-mini',
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


'''
input ex

y는 250, x는 0, z는 30으로 가게 해줘 그리고 y는 250, x는 100, z는 30으로 가게 해줘
'''