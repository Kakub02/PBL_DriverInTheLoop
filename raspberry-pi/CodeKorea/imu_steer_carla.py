#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from adafruit_servokit import ServoKit
from carla_msgs.msg import CarlaEgoVehicleControl

def imu_callback(data):
    #'imu' 메시지에서 'orientation' 필드만 출력
    orientation = data.orientation
    print("IMU orientation: x={}".format(orientation.x))
    print("IMU orientation: y={}".format(orientation.y))
    
    # 서보 각도를 [0, 180] 범위로 설정
    servo1_angle = max(0, min((orientation.x * 250 + 180) / 2, 180))
    servo2_angle = max(0, min((orientation.y * 400 + 180) / 2, 180))
    
    print("roll : ", servo1_angle)
    print("pitch : ", servo2_angle)
    
    kit.servo[7].angle = servo1_angle  # 서보 1 위치 설정 roll servo1
    kit.servo[6].angle = servo2_angle  # 서보 2 위치 설정 pitch
    
def steer_callback(data):
    steer = data.steer
    #print("Steer value: {}".format(steer))
    servo_angle = max(0, min((steer * 45 + 45), 90))
    #print("Servo angle: {}".format(servo_angle))
    print("=================================================================")
    kit.servo[4].angle = servo_angle

if __name__ == '__main__':
    kit = ServoKit(channels=16, address=0x40)  # PCA9685 생성
    rospy.init_node('imu_steer_orientation_subscriber', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/imu", Imu, imu_callback)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, steer_callback)
    rospy.spin()
