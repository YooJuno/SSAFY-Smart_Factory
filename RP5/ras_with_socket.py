# -*- coding: utf-8 -*-
import gpiod
import RPi.GPIO as GPIO
import socket
import time
import threading

# 서보 클래스
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
        self.set_angle(self.angle_center)

    def set_angle(self, angle):
        angle = max(self.angle_min, min(self.angle_max, angle))
        duty = 2 + (angle / 18)
        GPIO.output(self.servo_pin, True)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.3)
        GPIO.output(self.servo_pin, False)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# GPIO 설정
chip = gpiod.Chip('gpiochip0')
dir_line = chip.get_line(17)
step_line = chip.get_line(27)
enable_line = chip.get_line(22)

dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)

# 변수
motor_running = False
last_btn1 = 1
last_btn2 = 1
dir_line.set_value(0) 
enable_line.set_value(1) 
servo = ServoController()

# 스텝모터 루프
def step_motor_loop():
    global motor_running
    while True:
        if motor_running:
            step_line.set_value(1)
            time.sleep(0.0002)
            step_line.set_value(0)
            time.sleep(0.0002)
        else:
            time.sleep(0.01)

def connect_to_server(num1,num2):
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((num1, num2))
            return client_socket
        except socket.error:
            print("연결 실패. 5초 후 다시 시도")
            time.sleep(5)

# 스레드 시작
threading.Thread(target=step_motor_loop, daemon=True).start()
servo.set_angle(servo.angle_center)

client_socket = connect_to_server('192.168.110.110',40000)

try:
    while True:
        try:
            data = client_socket.recv(1024)
            if not data:
                break

            color = data.decode('utf-8')

            if color == "exit":
                break

            # 패널이 올려진 경우 → 토글 동작
            if color:
                motor_running = True
                enable_line.set_value(0 if motor_running else 1)
                print("Motor toggled:", "ON" if motor_running else "OFF")
                time.sleep(2.5)

            # red인 경우
            if color == "red":
                servo.set_angle(servo.angle_max)
                time.sleep(5.5)
            # 나머지
            else:
                servo.set_angle(servo.angle_min)
                time.sleep(5.5)

            servo.set_angle(servo.angle_center)
            time.sleep(1)
            motor_running = False
            enable_line.set_value(0 if motor_running else 1)
            print("Motor toggled:", "ON" if motor_running else "OFF")

        except socket.error as e:
            print(f"Socket error: {e}.")
            break

except KeyboardInterrupt:
    print("종료됨")

finally:
    motor_running = False
    enable_line.set_value(1)
    servo.cleanup()
    dir_line.release()
    step_line.release()
    enable_line.release()
