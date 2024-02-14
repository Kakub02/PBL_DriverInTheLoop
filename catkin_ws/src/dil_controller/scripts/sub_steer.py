#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit
from std_msgs.msg import String, Float32, Bool

# minimum and maximum steer angles
min_steer_angle = 60
max_steer_angle = 120

# pin number on pca9685 board
steer_pin = 2

# frequency in Hz
frequency = 50.0

class Steer_Control:
    def __init__(self):
        self.pca9685 = ServoKit(channels=16, address=0x40)  # PCA9685 
        self.steer_angle = 90

    def communication(self):
        rospy.Subscriber("/our_msg/steer", Float32, self.steer_callback)
        # rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, self.steer_callback)

    def steer_callback(self, msg):
        self.steer_angle = float(msg.data)
        if self.steer_angle > max_steer_angle:
            self.steer_angle = max_steer_angle
        elif self.steer_angle < min_steer_angle:
            self.steer_angle = min_steer_angle
        print("CARLA: steer: ", self.steer_angle)

    def run(self):
        self.communication()

        loop = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            self.pca9685.servo[steer_pin].angle = self.steer_angle
            # self.steer_servo.set_duty_cycle(self.steer_angle)
            loop.sleep()

    def stop(self):
        self.pca9685.servo[steer_pin].angle = 90


if __name__ == "__main__":
    rospy.init_node("steer_subscriber", anonymous=True)
    steer_control = Steer_Control()
    try:
        steer_control.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        steer_control.stop()
