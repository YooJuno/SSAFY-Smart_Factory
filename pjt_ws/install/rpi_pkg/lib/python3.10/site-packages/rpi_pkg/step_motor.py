# -*- coding: utf-8 -*-
import gpiod
import time
import threading

# GPIO pin numbers (BCM)
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22
BTN1_PIN = 23
BTN2_PIN = 24

# Open GPIO chip
chip = gpiod.Chip('/dev/gpiochip0')

# Get GPIO lines
dir_line = chip.get_line(DIR_PIN)
step_line = chip.get_line(STEP_PIN)
enable_line = chip.get_line(ENABLE_PIN)
btn1_line = chip.get_line(BTN1_PIN)
btn2_line = chip.get_line(BTN2_PIN)

# Request lines
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
btn1_line.request(consumer="btn1", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
btn2_line.request(consumer="btn2", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

# Motor control variables
motor_running = False
motor_direction = 0
target_direction = 0
TargetSpeed = 0.0001
InitialSpeed = 0.0005
Speed = InitialSpeed
RATIO = 0.0000005
isAccelerating = False
current_button = None

last_btn1_state = 1
last_btn2_state = 1

def step_motor():
    global motor_running, Speed, isAccelerating, target_direction
    while motor_running:
        if motor_direction != target_direction:
            if target_direction == 0:
                Speed += RATIO
                if Speed >= InitialSpeed:
                    motor_running = False
                    enable_line.set_value(1)
            else:
                isAccelerating = True
                Speed = InitialSpeed
        elif motor_direction == 0:
            Speed += RATIO
            if Speed >= InitialSpeed:
                motor_running = False
                enable_line.set_value(1)
        else:
            if isAccelerating:
                if Speed > TargetSpeed:
                    Speed -= RATIO
                else:
                    Speed = TargetSpeed
                    isAccelerating = False

        step_line.set_value(1)
        time.sleep(Speed)
        step_line.set_value(0)
        time.sleep(Speed)

def print_motor_status():
    print(f"[STATUS] Running: {motor_running}, Direction: {'CW' if motor_direction == 1 else 'CCW' if motor_direction == -1 else 'STOP'}, Speed: {Speed:.6f}")

try:
    while True:
        btn1_state = btn1_line.get_value()
        btn2_state = btn2_line.get_value()

        # Button 1 (CW)
        if btn1_state != last_btn1_state:
            print(f"[BUTTON 1] {'Pressed' if btn1_state == 0 else 'Released'}")
            if btn1_state == 0:
                if current_button == 1:
                    print("Stopping motor")
                    target_direction = 0
                    motor_direction = 0
                    current_button = None
                else:
                    print("Starting motor CW")
                    target_direction = 1
                    motor_direction = 1
                    dir_line.set_value(0)
                    isAccelerating = True
                    Speed = InitialSpeed
                    enable_line.set_value(0)
                    current_button = 1
                    if not motor_running:
                        motor_running = True
                        threading.Thread(target=step_motor).start()
                print_motor_status()

        # Button 2 (CCW)
        if btn2_state != last_btn2_state:
            print(f"[BUTTON 2] {'Pressed' if btn2_state == 0 else 'Released'}")
            if btn2_state == 0:
                if current_button == 2:
                    print("Stopping motor")
                    target_direction = 0
                    motor_direction = 0
                    current_button = None
                else:
                    print("Starting motor CCW")
                    target_direction = -1
                    motor_direction = -1
                    dir_line.set_value(1)
                    isAccelerating = True
                    Speed = InitialSpeed
                    enable_line.set_value(0)
                    current_button = 2
                    if not motor_running:
                        motor_running = True
                        threading.Thread(target=step_motor).start()
                print_motor_status()

        last_btn1_state = btn1_state
        last_btn2_state = btn2_state

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated by user.")

finally:
    motor_running = False
    time.sleep(0.1)
    enable_line.set_value(1)
    dir_line.release()
    step_line.release()
    enable_line.release()
    btn1_line.release()
    btn2_line.release()
