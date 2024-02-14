#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit

from std_msgs.msg import String, Float32, Bool


from dil_controller.mpu6050 import *
from dil_controller.pid_controller import *


# minimum and maximum angles of roll and pitch
min_roll_angle = 70
max_roll_angle = 130

min_pitch_angle = 65
max_pitch_angle = 120

# pin numbers on pca9685 board
roll_pin = 0
pitch_pin = 1

# frequency in Hz
frequency = 50.0

class Servo_Control:
    def __init__(self):
        # self.mpu = mpu6050(0x68)
        # self.PID_servo1 = PIDController(1, 0.1, 0.01)
        # self.PID_servo2 = PIDController(1, 0.1, 0.01)
        self.pca9685 = ServoKit(channels=16, address=0x40)  # PCA9685 

        self.roll_angle = 90
        self.pitch_angle = 90

    def communication(self):
        rospy.Subscriber("/our_msg/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("/our_msg/roll", Float32, self.roll_callback)

    def pitch_callback(self, msg):
        self.pitch_angle = float(msg.data) + 90
        if self.pitch_angle > max_pitch_angle:
            self.pitch_angle = max_pitch_angle
        elif self.pitch_angle < min_pitch_angle:
            self.pitch_angle = min_pitch_angle
        print("CARLA: pitch: ", self.pitch_angle)

    def roll_callback(self, msg):
        self.roll_angle = float(msg.data) + 100
        if self.roll_angle > max_roll_angle:
            self.roll_angle = max_roll_angle
        elif self.roll_angle < min_roll_angle:
            self.roll_angle = min_roll_angle
        print("CARLA: roll: ", self.roll_angle)

    def run(self):
        self.communication()

        loop = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            self.pca9685.servo[roll_pin].angle = self.roll_angle
            self.pca9685.servo[pitch_pin].angle = self.pitch_angle

            loop.sleep()

    def stop(self):
        self.pca9685.servo[roll_pin].angle = 100
        self.pca9685.servo[pitch_pin].angle = 90

if __name__ == "__main__":
    rospy.init_node("imu_steer_orientation_subscriber", anonymous=True)
    servo_control = Servo_Control()
    try:
        servo_control.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        servo_control.stop()
