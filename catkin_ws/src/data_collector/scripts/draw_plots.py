#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
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
        speed = float(msg.data)
        self.speed_data.append(speed)

    def steer_callback(self, msg):
        steer = float(msg.data)
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
        #pzlt.ion
        rate = rospy.Rate(50)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            self.plot_data()
            #plt.pause(0.05)
            rate.sleep()

    def plot_data(self):
        fig, axs = plt.subplots(5, 1, sharex=True, figsize=(10, 8))

        axs[0].plot(self.speed_data, label='Speed')
        axs[0].set_ylabel('Speed')
        axs[0].legend()

        axs[1].plot(self.steer_data, label='Steer Angle')
        axs[1].set_ylabel('Steer Angle')
        axs[1].legend()

        axs[2].plot(self.roll_data, label='Roll')
        axs[2].set_ylabel('Roll')
        axs[2].legend()

        axs[3].plot(self.pitch_data, label='Pitch')
        axs[3].set_ylabel('Pitch')
        axs[3].legend()

        axs[4].plot(self.reverse_data, label='Reverse')
        axs[4].set_xlabel('Time')
        axs[4].set_ylabel('Reverse')
        axs[4].legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    try:
        plotting_node = PlottingNode()
        plotting_node.run()
    finally:
            plt.close()
        
