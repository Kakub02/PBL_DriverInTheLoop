#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Imu
from carla_msgs.msg import CarlaEgoVehicleControl

class PlottingNode:
    def __init__(self):
        rospy.init_node("plotting_node", anonymous=True)
        rospy.Subscriber("/our_msg/speed", Float32, self.speed_callback)
        rospy.Subscriber("/our_msg/steer", Float32, self.steer_callback)
        rospy.Subscriber("/our_msg/roll", Float32, self.roll_callback)
        rospy.Subscriber("/our_msg/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("/our_msg/reverse", Bool, self.reverse_callback)

        self.speed_data = []
        self.steer_data = []
        self.roll_data = []
        self.pitch_data = []
        self.reverse_data = []

    def speed_callback(self, msg):
        speed = msg.data
        self.speed_data.append(speed)

    def steer_callback(self, msg):
        steer = msg.data
        self.steer_data.append(steer)

    def roll_callback(self, msg):
        roll = msg.data
        self.roll_data.append(roll)

    def pitch_callback(self, msg):
        pitch = msg.data
        self.pitch_data.append(pitch)

    def reverse_callback(self, msg):
        reverse = msg.data
        self.reverse_data.append(reverse)

    def run(self):
        rate = rospy.Rate(50)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            self.plot_data()
            rate.sleep()

    def plot_data(self):
        plt.figure(1)

        plt.subplot(511)
        plt.plot(self.speed_data, label='Speed')
        plt.title('Car Data over Time')
        plt.xlabel('Time')
        plt.ylabel('Speed')
        plt.legend()

        plt.subplot(512)
        plt.plot(self.steer_data, label='Steer Angle')
        plt.xlabel('Time')
        plt.ylabel('Steer Angle')
        plt.legend()

        plt.subplot(513)
        plt.plot(self.roll_data, label='Roll')
        plt.xlabel('Time')
        plt.ylabel('Roll')
        plt.legend()

        plt.subplot(514)
        plt.plot(self.pitch_data, label='Pitch')
        plt.xlabel('Time')
        plt.ylabel('Pitch')
        plt.legend()

        plt.subplot(515)
        plt.plot(self.reverse_data, label='Reverse')
        plt.xlabel('Time')
        plt.ylabel('Reverse')
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    plotting_node = PlottingNode()
    plotting_node.run()
