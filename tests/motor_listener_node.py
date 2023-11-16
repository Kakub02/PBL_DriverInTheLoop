#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String, Int32
import RPi.GPIO as GPIO
from time import sleep

# dc motor pins 
in1 = 24
in2 = 23
en = 25

# dc motor setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
pwm = GPIO.PWM(en, 1000)
pwm.start(0)

def set_speed(speed):
    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100
    else:
        pwm.ChangeDutyCycle(speed)

def speed_direction_callback(data):
    print(data)
    speed_input, is_reverse_input = data.data.split(',')
    speed = int(speed_input)
    is_reverse = bool(int(is_reverse_input))



    if reverse:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        print("backward")
    else:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        print("forward")
    rospy.loginfo(f"Received speed: {speed}, is reverse: {is_reverse}")
#     speed_kmph = float(speed)

#     max_motor_speed_kmph = 10.0  # trza to dostosować

#     pwm_value = speed_to_pwm(speed_kmph, max_motor_speed_kmph, pwm_range)

#     set_motor_speed(pwm_value, reverse)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, speed_direction_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        GPIO.cleanup()