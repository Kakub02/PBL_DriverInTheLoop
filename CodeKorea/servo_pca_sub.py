#!/usr/bin/env python3

import rospy
from servo_control.msg import ServoPosition
import RPi.GPIO as GPIO 
from time import sleep
from adafruit_servokit import ServoKit
from pid_controller import PIDController
from mpu6050 import mpu6050

def servo_callback(data):
    rospy.loginfo("Received servo positions: %f, %f", data.position1, data.position2)
    
    gyro_data = mpu.get_gyro_data()
    X_mpu = gyro_data['x']
    Y_mpu = gyro_data['y']
    error1 = PID_servo1.calculate(data.position1, X_mpu)
    error2 = PID_servo2.calculate(data.position2, Y_mpu)
    
    kit.servo[7].angle = data.position1 + error1 # 서보 1 위치 설정 there should be + or -???
    kit.servo[8].angle = data.position2 + error2  # 서보 2 위치 설정


def servo_subscriber():
    rospy.init_node('servo_subscriber', anonymous=True)
    rospy.Subscriber('/servo_position', ServoPosition, servo_callback)
    rospy.spin()


if __name__ == '__main__':
    kit = ServoKit(channels=16, address=0x40)  # PCA9685 객체 생성
    PID_servo1 = PIDController(1, 0.1, 0.01)
    PID_servo2 = PIDController(1, 0.1, 0.01)
    mpu = mpu6050(0x68)
    try:
        servo_subscriber()
    except rospy.ROSInterruptException:
        pass