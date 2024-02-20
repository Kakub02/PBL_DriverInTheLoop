#!/usr/bin/env python3

import math as m
import rospy
from adafruit_servokit import ServoKit
from std_msgs.msg import String, Float32, Bool

from dil_controller.mpu6050 import *
from dil_controller.pid_controller import *
from dil_controller.real_time_plot import *

# minimum and maximum angles of roll and pitch
min_roll_angle = 60
max_roll_angle = 120
roll_servo_shift = 10

min_pitch_angle = 65
max_pitch_angle = 120
pitch_servo_shift = 0

# pin numbers on pca9685 board
roll_pin = 0
pitch_pin = 1

# frequency in Hz
alpha = 0.98
frequency = 50.0  # in Hz
dt = 1 / frequency

def map_pitch_to_servo(pitch_angle):
    pitch_angle = round(pitch_angle + 90.0 + pitch_servo_shift, 2)
    if pitch_angle > max_roll_angle + pitch_servo_shift:
        pitch_angle = max_roll_angle + pitch_servo_shift
    elif pitch_angle < min_pitch_angle + pitch_servo_shift:
        pitch_angle = min_pitch_angle + pitch_servo_shift
    return pitch_angle

def map_roll_to_servo(roll_angle):
    roll_angle = round(roll_angle + 90.0 + roll_servo_shift, 2)
    if roll_angle > max_roll_angle + roll_servo_shift:
        roll_angle = max_roll_angle + roll_servo_shift
    elif roll_angle < min_roll_angle + roll_servo_shift:
        roll_angle = min_roll_angle + roll_servo_shift
    return roll_angle

class Servo_Control:
    def __init__(self):
        self.labels = ['carla_roll', 'carla_pitch', 'imu_roll', 'imu_pitch']

        self.real_time_plot = RealTimePlot(self.labels, 'Carla roll and pitch vs imu roll and pitch over Time', 'Samples', 'Values', 200, (-180, 180), 50)
        self.real_time_plot.set_updating_function(lambda: self.run())

        self.pca9685 = ServoKit(channels=16, address=0x40)  # PCA9685 
        self.mpu = mpu6050(0x68)

        self.carla_roll = 90 + roll_servo_shift
        self.carla_pitch = 90 + pitch_servo_shift

        self.imu_final_roll = 0.0
        self.imu_final_pitch = 0.0

    def communication(self):
        rospy.Subscriber("/our_msg/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("/our_msg/roll", Float32, self.roll_callback)

    def pitch_callback(self, msg):
        self.carla_pitch = float(msg.data)
        print("CARLA: pitch: ", self.carla_pitch)

    def roll_callback(self, msg):
        self.carla_roll = float(msg.data)
        print("CARLA: roll: ", self.carla_roll)

    def run(self):
        self.communication()

        loop = rospy.Rate(frequency)

        while not rospy.is_shutdown():

            accel_data = self.mpu.get_accel_data()
            gyro_data = self.mpu.get_gyro_data()

            # Accelerometer-derived angles
            a_roll = round(m.degrees(m.atan2(accel_data["y"], m.sqrt(accel_data["x"] ** 2 + accel_data["z"] ** 2))), 2)
            a_pitch = round(m.degrees(m.atan2((-1)*accel_data["x"], m.sqrt(accel_data["y"] ** 2 + accel_data["z"] ** 2))), 2)

            # Gyroscope-derived angles
            g_roll = gyro_data["x"] * dt
            g_pitch =   gyro_data["y"] * dt

            # Complementary filter
            self.imu_final_roll = alpha * (self.imu_final_roll + g_roll) + (1 - alpha) * a_roll
            self.imu_final_pitch = alpha * (self.imu_final_pitch + g_pitch) + (1 - alpha) * a_pitch

            # updating plot data
            with self.real_time_plot.data_lock:
                self.real_time_plot.y_data[self.real_time_plot.lines_labels[0]].append(self.carla_roll)
                self.real_time_plot.y_data[self.real_time_plot.lines_labels[1]].append(self.carla_pitch)
                self.real_time_plot.y_data[self.real_time_plot.lines_labels[2]].append(self.imu_final_roll)
                self.real_time_plot.y_data[self.real_time_plot.lines_labels[3]].append(self.imu_final_pitch)

            self.pca9685.servo[roll_pin].angle = map_roll_to_servo(self.carla_roll)
            self.pca9685.servo[pitch_pin].angle = map_pitch_to_servo(self.carla_pitch)
            loop.sleep()

    def stop(self):
        self.pca9685.servo[roll_pin].angle = 90 + roll_servo_shift
        self.pca9685.servo[pitch_pin].angle = 90 + pitch_servo_shift

if __name__ == "__main__":
    rospy.init_node("imu_steer_orientation_subscriber", anonymous=True)
    servo_control = Servo_Control()
    try:
        servo_control.real_time_plot.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        servo_control.stop()
