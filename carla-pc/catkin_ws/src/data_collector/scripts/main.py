#!/usr/bin/env python3
import rospy
import math as m

from std_msgs.msg import String, Float32, Bool, Int32
from sensor_msgs.msg import Imu

#pobieramy CarlaEgoVehicleControl - steer
from carla_msgs.msg import CarlaEgoVehicleControl

alpha = 0.98
frequency = 50.0
dt = 1 / frequency

class MainNode:  
    def __init__(self):
        self.current_speed_pub = 0
        self.current_reverse_pub = 0
        self.current_roll_deg_pub = 0
        self.current_pitch_deg_pub = 0
        self.current_steer_pub = 0
        self.final_roll = 0.0
        self.final_pitch = 0.0

    def comunication(self) -> None:
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.current_orientation_callback)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, self.current_speed_callback)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, self.current_steer_callback)

        self.current_speed_pub = rospy.Publisher("/our_msg/speed", Float32, queue_size=10)
        self.current_reverse_pub = rospy.Publisher("/our_msg/reverse", Bool, queue_size=10)
        self.current_roll_deg_pub = rospy.Publisher("/our_msg/roll", Float32, queue_size=10)
        self.current_pitch_deg_pub = rospy.Publisher("/our_msg/pitch", Float32, queue_size=10)
        self.current_steer_pub= rospy.Publisher("/our_msg/steer", Float32, queue_size=10)

    def current_speed_callback(self, msg):
        speed = msg.data
        speed_calc = 3.6*float(speed)
        reverse_flag = False
        if speed_calc < 0:
            speed_calc = speed_calc * (-1)
            reverse_flag = True
        
        self.current_speed_pub.publish(speed_calc)
        self.current_reverse_pub.publish(reverse_flag)
    
    def current_steer_callback(self,msg:CarlaEgoVehicleControl):
        steer = float(msg.steer) #msg.data
        steer_angle = max(30, min(steer * 100 + 90, 150))
        
        self.current_steer_pub.publish(steer_angle)

    def current_orientation_callback(self, msg: Imu):
        #acc
        acceleration_x = float(msg.linear_acceleration.x) # m/s^2
        acceleration_y = float(msg.linear_acceleration.y)
        acceleration_z = float(msg.linear_acceleration.z)

        #gyro
        gyro_x = float(msg.angular_velocity.x) # deg/s
        gyro_y = float(msg.angular_velocity.y)
        gyro_z = float(msg.angular_velocity.z)

        # Accelerometer-derived angles
        a_roll = round(m.degrees(m.atan2(acceleration_y, m.sqrt(acceleration_x ** 2 + acceleration_z ** 2))), 2)
        a_pitch = round(m.degrees(m.atan2((-1)*acceleration_x, m.sqrt(acceleration_y ** 2 + acceleration_z ** 2))), 2)

        # Gyroscope-derived angles
        g_roll = gyro_x * dt
        g_pitch = gyro_y * dt

        # Complementary filter
        self.final_roll = alpha * (self.final_roll + g_roll) + (1 - alpha) * a_roll
        self.final_pitch = alpha * (self.final_pitch + g_pitch) + (1 - alpha) * a_pitch

        self.current_roll_deg_pub.publish(self.final_roll)
        self.current_pitch_deg_pub.publish(self.final_pitch)

    # A to żeby nam działało
    def run(self):
        self.comunication()
        loop = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            loop.sleep()

    # Odpalanie tego gnojka
if __name__ == "__main__":
    rospy.init_node("Main_Node", anonymous=True)
    main = MainNode()
    main.run()
