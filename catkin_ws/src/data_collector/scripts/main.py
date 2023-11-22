#!/usr/bin/env python3
import rospy
import math

from std_msgs.msg import String, Float32, Bool, Int32
from sensor_msgs.msg import Imu


#pobieramy CarlaEgoVehicleControl - steer
from carla_msgs.msg import CarlaEgoVehicleControl


class MainNode:  
    #-> None działa jak Void w c++
    def init_communication(self) -> None:
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
        speed_calc = 4*float(speed)
        reverse_flag = bool(False)
        if speed_calc < 0:
            speed_calc = speed_calc * (-1)
            reverse_flag = bool(True)
        
        self.current_speed_pub.publish(speed_calc)
        self.current_reverse_pub.publish(reverse_flag)
    
    def current_steer_callback(self,msg:CarlaEgoVehicleControl):
        steer = msg.steer #msg.data
        steer = float(steer)
        if steer <= 0:
            servo_angle = max(-50, steer * 90 + 90)
        else:
            servo_angle = max(0, min(steer * 90 + 90, 50))
        
        self.current_steer_pub.publish(servo_angle)


    def current_orientation_callback(self, msg: Imu):
        orientaion_x = float(msg.linear_acceleration.x)
        orientaion_y = float(msg.linear_acceleration.y)
        orientaion_z = float(msg.linear_acceleration.z)

        roll, pitch = self.calculate_roll_and_pitch(orientaion_x, orientaion_y, orientaion_z)

        self.current_roll_deg_pub.publish(roll)
        self.current_pitch_deg_pub.publish(pitch)
    
    def calculate_roll_and_pitch(self, orientaion_x, orientaion_y, orientaion_z):
        roll_rad = math.atan2(orientaion_y, math.sqrt(orientaion_x**2 + orientaion_z**2))
        pitch_rad = math.atan2(orientaion_x, math.sqrt(orientaion_y**2 + orientaion_z**2))

        pitch_deg = math.degrees(pitch_rad)
        roll_deg = math.degrees(roll_rad)

        return roll_deg, pitch_deg


    # A to żeby nam działało
    def run(self):
        self.init_communication()
        loop = rospy.Rate(25.0) # frequency in Hz
        while not rospy.is_shutdown():
            loop.sleep()
            
    # Odpalanie tego gnojka
if __name__ == "__main__":
    rospy.init_node("Main_Node", anonymous=True)
    main = MainNode()
    main.run()
