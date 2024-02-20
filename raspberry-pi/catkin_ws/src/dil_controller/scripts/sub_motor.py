#!/usr/bin/env

import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String, Float32, Bool
from dil_controller.dc_motor import *
from dil_controller.encoder import *
from dil_controller.pid_controller import *

# Set GPIO pin numbers for motors
# Motor 1
motor_in1 = 22
motor_in2 = 27
motor_ena = 17
# Motor 2
motor_in3 = 23
motor_in4 = 24
motor_enb = 25

# Set GPIO pin numbers for encoder
left_encoder_pin = 16
right_encoder_pin = 12

# Set PID parametersD
Kp = 0.1538
Ki = 2.263
Kd = 0

# Set border values
min_speed_kmh = 0.1
max_speed_kmh = 60

# Border pwm values
min_pwm = 0
max_pwm = 100

# v max of encoder in impulses/second
max_encoder_speed = 10000 # on frequency 50Hz

# frequency in Hz
frequency = 50.0

# map speed from kilometers/hour to impulses/second
def speed_kmh_to_imps(speed):
    return speed / max_speed_kmh * max_encoder_speed

class Motor_Control:
    def __init__(self):
        self.motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
        self.motor2 = DC_Motor(motor_in3, motor_in4, motor_enb)
        self.encoder = Encoder(left_encoder_pin,right_encoder_pin)
        self.pid = PID_controller(Kp, Ki, Kd, 1, 0, 100, 0, 250, 1/frequency)

        self.setpoint_speed = 0
        self.reverse = 0  # forward

    def communication(self):
        rospy.Subscriber("/our_msg/speed", Float32, self.speed_callback)
        rospy.Subscriber("/our_msg/reverse", Bool, self.reverse_callback)

    def speed_callback(self, msg):
        speed_kmh = float(msg.data)
        if speed_kmh > max_speed_kmh:
            speed_kmh = max_speed_kmh
        elif speed_kmh < min_speed_kmh:
            speed_kmh = 0
        self.setpoint_speed = speed_kmh_to_imps(speed_kmh)
        print("CARLA: setpoint_speed = ",self.setpoint_speed)

    def reverse_callback(self, msg):
        self.reverse = bool(msg.data)
        print("CARLA: is_reverse = ",self.reverse)

    def run(self):
        self.communication()

        loop = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            encoder_value = self.encoder.get_value()
            self.encoder.clear_value()

            real_speed = encoder_value * frequency
            pid_pwm = self.pid.update(self.setpoint_speed, real_speed)

            self.motor1.set_speed(pid_pwm, self.reverse)
            self.motor2.set_speed(pid_pwm, self.reverse)
            
            loop.sleep()

    def stop(self):
        self.motor1.stop_motor()
        self.motor2.stop_motor()


if __name__ == "__main__":
    rospy.init_node("Motor_node", anonymous=True)
    motor_control = Motor_Control()
    try:
        motor_control.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_control.stop()
