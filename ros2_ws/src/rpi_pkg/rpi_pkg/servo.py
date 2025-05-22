import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import re  # 문자열에서 숫자 추출용

class ServoController:
    def __init__(self, pin=18, angle_min=75, angle_max=130, angle_center=103):
        self.servo_pin = pin
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_center = angle_center

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)

    def set_angle(self, angle):
        if not self.angle_min <= angle <= self.angle_max:
            print(f'Invalid input: {angle}. Valid range is {self.angle_min}–{self.angle_max}')
        angle = min(max(self.angle_min, angle), self.angle_max)
        print(f"Setting angle: {angle}")
        duty = 2 + (angle / 18)
        GPIO.output(self.servo_pin, True)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        GPIO.output(self.servo_pin, False)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.servo = ServoController()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # 예: "angle: 110" → 숫자 추출
        match = re.search(r'\d+', msg.data)
        if match:
            angle = int(match.group())
            self.servo.set_angle(angle)

    def destroy_node(self):
        self.servo.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
