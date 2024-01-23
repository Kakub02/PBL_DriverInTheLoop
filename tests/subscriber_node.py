#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from motor import Motor
from pid_controller import PIDController
from tests.encoder_test import Encoder


# Set GPIO pin numbers for motor
motor_in1 = 23
motor_in2 = 24
motor_en = 25

# Set GPIO pin numbers for encoder
left_encoder_pin = 17
right_encoder_pin = 18

# Set PID parameters
kp=1.0
ki=0.1
kd=0.01

# Set maximum speed in kmph
max_speed = 60

class MotorListenerNode:
    def __init__(self):
        rospy.init_node('motor_listener_node', anonymous=True)

        self.motor = Motor(motor_in1, motor_in2, motor_en)

        self.encoder = Encoder(left_encoder_pin, right_encoder_pin)

        self.pid_controller = PIDController(kp, ki, kd)


        # Set the loop rate (adjust as needed)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribe to the setpoint speed topic
        rospy.Subscriber('setpoint_speed', Float32, self.setpoint_speed_callback)

    def setpoint_speed_callback(self, data):
        setpoint_speed, reverse = data.data.split(',')

        setpoint_speed_fraction = setpoint_speed / max_speed

        while not rospy.is_shutdown(): # kiedy dostaniemy nowy message przerwij petle
            # Calculate the actual speed from the encoder
            actual_speed_fraction = self.encoder.getRealSpeed()
            rospy.loginfo(f"Setpoint Speed in %: {setpoint_speed_fraction*100}")
            rospy.loginfo(f"Current Speed in %: {actual_speed_fraction*100}")

            # Use PID controller to calculate the adjusted speed
            adjusted_speed = self.pid_controller.calculate(setpoint_speed_fraction, actual_speed_fraction)

            # Adjust the motor speed
            self.motor.setSpeed(adjusted_speed * 100, not reverse)
            self.rate.sleep() # musimy zrobić sleep żeby każda iteracja pida była mniej wiecej w rownym odstepie czasu

if __name__ == '__main__':
    motor_listener_node = MotorListenerNode()
