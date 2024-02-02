#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit

from std_msgs.msg import String, Float32, Bool

# from dil_controller_src.mpu6050 import mpu6050
# from dil_controller_src.pid_controller import PID_controller


min_roll_angle = 70
max_roll_angle = 130

min_pitch_angle = 65
max_pitch_angle = 120

min_steer_angle = 60  # ????????
max_steer_angle = 120  # ????????

roll_pin = 0
pitch_pin = 1
#steer_pin = 

class PID_controller:
    # Controller gains 
    Kp = 0.0
    Ki = 0.0
    Kd = 0.0

   	# Derivative low-pass filter time constant
    tau = 0.0

    # Output limits
    lower_limit = 0.0
    upper_limit = 0.0

    # Integrator limits
    lower_int_limit = 0.0
    upper_int_limit = 0.0

	# Sample time (in seconds)
    T = 0.0

    # Controller memory
    integrator = 0.0
    previous_error = 0.0 # Required for integrator
    differentiator = 0.0
    previous_measurement = 0.0 # Required for differentiator

    def __init__(self, Kp, Ki, Kd, tau, lower_limit, upper_limit, lower_int_limit, upper_int_limit, T) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.tau = tau # 1/2 sekundy

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        self.lower_int_limit = lower_int_limit
        self.upper_int_limit = upper_int_limit
        
        self.T = T

    # Function for calculating actual control value
    def update(self, setpoint, measurement):
        
        # Error signal
        error = setpoint - measurement
        
        # Proportional
        proportional = self.Kp * error
    
        # Integral
        self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.previous_error)

	    # Anti-wind-up via integrator clamping
        if (self.integrator > self.upper_int_limit): # -100----200/300
            self.integrator = self.upper_int_limit
        elif (self.integrator < self.lower_int_limit):
            self.integrator = self.lower_int_limit

        # Derivative (band-limited differentiator)
        # Note: derivative on measurement, therefore minus sign in front of equation!
        self.differentiator = -(2.0 * self.Kd * (measurement - self.previous_measurement) + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)


        # Compute output and apply limits
        output = proportional + self.integrator + self.differentiator

        if (output > self.upper_limit):
            output = self.upper_limit
        elif (output < self.lower_limit):
            output = self.lower_limit

	    # Store error and measurement for later use
        self.previous_error = error
        self.previous_measurement = measurement

        return output

class Servo_Control:
    def __init__(self):
        # self.mpu = mpu6050(0x68)
        # self.PID_roll_servo = PIDController(1, 0.1, 0.01)
        # self.PID_pitch_servo = PIDController(1, 0.1, 0.01)
        self.pca9685 = ServoKit(channels=16, address=0x40)  # PCA9685 

        self.roll_angle = 90
        self.pitch_angle = 90
        self.steer_angle = 90

    def communication(self):
        rospy.Subscriber("/our_msg/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("/our_msg/roll", Float32, self.roll_callback)
        # rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, self.steer_callback)

    def pitch_callback(self, msg):
        self.pitch_angle = float(msg.data) + 90
        if self.pitch_angle > max_pitch_angle:
            self.pitch_angle = max_pitch_angle
        elif self.pitch_angle < min_pitch_angle:
            self.pitch_angle = min_pitch_angle
        print("pitch: ", self.pitch_angle)

    def roll_callback(self, msg):
        self.roll_angle = float(msg.data) + 100
        if self.roll_angle > max_roll_angle:
            self.roll_angle = max_roll_angle
        elif self.roll_angle < min_roll_angle:
            self.roll_angle = min_roll_angle
        print("roll: ", self.roll_angle)

    # def steer_callback(self, msg):
    #     steer_angle = float(msg.data)
    #     if self.steer_angle > max_steer_angle:
    #         self.steer_angle = max_steer_angle
    #     elif self.steer_angle < min_steer_angle:
    #         self.roll_angle = min_steer_angle
    #     print("roll: ", self.steer_angle)

    def run(self):
        self.communication()

        loop = rospy.Rate(50.0)  # frequency in Hz

        while not rospy.is_shutdown():
            self.pca9685.servo[roll_pin].angle = self.roll_angle
            self.pca9685.servo[pitch_pin].angle = self.pitch_angle
            # self.steer_servo.set_duty_cycle(self.steer_angle)
            loop.sleep()


if __name__ == "__main__":
    rospy.init_node("imu_steer_orientation_subscriber", anonymous=True)
    servo_control = Servo_Control()
    servo_control.run()
