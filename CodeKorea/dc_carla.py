#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

ENA = 26
IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5
ENB = 0

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM 설정
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def speed_callback(data):
    speed = data.data
    rospy.loginfo("Get Speed : {}".format(speed))

    # 속도를 PWM 듀티 사이클로 매핑합니다.
    duty_cycle_a = map_speed_to_duty_cycle(speed)
    duty_cycle_b = duty_cycle_a

    # 각각의 모터에 PWM 듀티 사이클을 설정합니다.
    pwm_a.ChangeDutyCycle(duty_cycle_a)
    pwm_b.ChangeDutyCycle(duty_cycle_b)

    # 속도의 부호에 따라 모터 방향을 제어합니다.
    if speed > 0:
        # 정방향
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif speed < 0:
        # 역방향
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)

def map_speed_to_duty_cycle(speed):
    min_speed = -15.0
    max_speed = 15.0
    min_duty_cycle = 0
    max_duty_cycle = 100
    
    # 입력 속도를 min_speed와 max_speed 사이로 클리핑합니다.
    speed = max(min(speed, max_speed), min_speed)
    
    # 속도에 따라 PWM 듀티 사이클을 계산합니다.
    if speed == 0:
        duty_cycle = 0
    else:
        duty_cycle = ((speed - min_speed) / (max_speed - min_speed)) * (max_duty_cycle - min_duty_cycle) + min_duty_cycle
    print("duty cycle : ", duty_cycle)
    return duty_cycle

if __name__ == '__main__':
    try:
        rospy.init_node('motor_controller', anonymous=True)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, speed_callback)
        rospy.spin()

    finally:
        GPIO.cleanup()