#!/usr/bin/env python3
import rospy
import math

from std_msgs.msg import String, Float32, Bool, Int32
from geometry_msgs.msg import Accel


#pobieramy CarlaEgoVehicleStatus -  float32 velocity, geometry_msgs/Accel acceleration (tego nie trzeba, jeżeli użyjemy imu)
#pobieramy CarlaEgoVehicleControl - bool reverse
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleControl


class MainNode:  
    #-> None działa jak Void w c++
    def init_communication(self) -> None:
        #rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.current_velocity_callback)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.current_acceleration_callback)
        #rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.current_acceleration_callback)
        #rospy.Subscriber("/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleControl, self.current_reverse_callback)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, self.current_speed_callback)
        #self.current_velocity_pub = rospy.Publisher("/our_msg/current_velocity", Float32, queue_size=10)
        self.current_speed_pub = rospy.Publisher("/our_msg/speed", Float32, queue_size=10)
        self.current_roll_deg_pub = rospy.Publisher("/our_msg/roll", Float32, queue_size=10)
        self.current_pitch_deg_pub = rospy.Publisher("/our_msg/pitch", Float32, queue_size=10)
        #self.current_roll_deg_pub_x = rospy.Publisher("/our_msg/acceleration_x", Float32, queue_size=10)
        #self.current_roll_deg_pub_y = rospy.Publisher("/our_msg/acceleration_y", Float32, queue_size=10)
        #self.current_roll_deg_pub_z = rospy.Publisher("/our_msg/acceleration_z", Float32, queue_size=10)

        self.current_reverse_pub = rospy.Publisher("/our_msg/reverse", Bool, queue_size=10)

    def current_speed_callback(self, msg):

        speed = msg.data
        speed_calc = float(speed)
        reverse_flag = bool(False)
        if speed_calc < 0:
            speed_calc = speed_calc * (-1)
            reverse_flag = bool(True)
        
        self.current_speed_pub.publish(speed_calc)
        self.current_reverse_pub.publish(reverse_flag)

    # Zmieniamy prędkośc na float
    def current_velocity_callback(self, msg: CarlaEgoVehicleStatus):
        self.current_velocity_pub.publish(Float32(msg.velocity))

    # Acceleration zmienamy na trzy składowe, x i y i z, będą one nam potrzebne do obliczeń obrotu
    def current_acceleration_callback(self, msg: CarlaEgoVehicleStatus):
        # IDK, czy to ma być msg.acceleration.linear.x, czy msg.linear.x
        acceleration_x = float(msg.acceleration.linear.x)
        acceleration_y = float(msg.acceleration.linear.y)
        acceleration_z = float(msg.acceleration.linear.z)

        roll, pitch = self.calculate_roll_and_pitch(acceleration_x, acceleration_y, acceleration_z)

        self.current_roll_deg_pub.publish(roll)
        self.current_pitch_deg_pub.publish(pitch)
        #self.current_roll_deg_pub_x.publish(acceleration_x)
        #self.current_roll_deg_pub_y.publish(acceleration_y)
        #self.current_roll_deg_pub_z.publish(acceleration_z)

    
    def calculate_roll_and_pitch(self, acceleration_x, acceleration_y, acceleration_z):
        # Obliczamy wychylenia
        pitch_rad = math.atan2(acceleration_y, math.sqrt(acceleration_x**2 + acceleration_z**2))

        # Obliczamy kąt przechylenia
        roll_rad = math.atan2(acceleration_x, math.sqrt(acceleration_y**2 + acceleration_z**2))

        # Zamieniamy radiany na stopnie
        pitch_deg = math.degrees(pitch_rad)
        roll_deg = math.degrees(roll_rad)

        return roll_deg, pitch_deg


    # A to żeby nam działało
    def run(self):
        self.init_communication()
        loop = rospy.Rate(10.0) # frequency in Hz
        while not rospy.is_shutdown():
            loop.sleep()
            
    # Odpalanie tego gnojka
if __name__ == "__main__":
    rospy.init_node("Main_Node", anonymous=True)
    main = MainNode()
    main.run()
