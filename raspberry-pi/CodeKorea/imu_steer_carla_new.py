#!/usr/bin/env python3

import rospy
#from servo_control.msg import ServoPosition
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Imu
#import RPi.GPIO as GPIO 
#from time import sleep
from adafruit_servokit import ServoKit
from tests.pid_controller import PIDController
from tests.mpu6050 import mpu6050

lower_limit = 60 #min angle of roll and pitch
upper_limit = 120 #max angle of roll and pitch

class Servo_Control():
    def __init__(self):
        self.kit = ServoKit(channels=16, address=0x40)  # PCA9685 객체 생성
        self.mpu = mpu6050(0x68)
        self.PID_servo1 = PIDController(1, 0.1, 0.01)
        self.PID_servo2 = PIDController(1, 0.1, 0.01)

        rospy.init_node('imu_steer_orientation_subscriber', anonymous=True)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, self.steer_callback)
        rospy.spin()

    def imu_callback(self, data):
        #'imu' 메시지에서 'orientation' 필드만 출력
        orientation = data.orientation
        print("IMU orientation: x={}".format(orientation.x))
        print("IMU orientation: y={}".format(orientation.y))
        
        #servo1_angle = max(0, min((orientation.x * 250 + 180) / 2, 180))
        
        servo1_angle = (orientation.x * 250 + 180) / 2
        if servo1_angle >= upper_limit:
            servo1_angle = upper_limit
        elif servo1_angle <= lower_limit:
            servo1_angle = lower_limit
        #servo2_angle = max(0, min((orientation.y * 400 + 180) / 2, 180))
        servo2_angle = (orientation.y * 400 + 180) / 2
        if servo2_angle >= upper_limit:
            servo2_angle = upper_limit
        elif servo2_angle <= lower_limit:
            servo2_angle = lower_limit

        print("roll : ", servo1_angle)
        print("pitch : ", servo2_angle)

        # get actual gyroscope data
        gyro_data = self.mpu.get_gyro_data()
        X_mpu = gyro_data['x']
        Y_mpu = gyro_data['y']

        print("gyro x: ", X_mpu)
        print("gyro y: ", Y_mpu)

        # use PI control
        cv1 = self.PID_servo1.calculate(servo1_angle, X_mpu) # roll
        cv2 = self.PID_servo2.calculate(servo2_angle, Y_mpu) # pitch
        
        print("PID calculated value roll", cv1)
        print("PID calculated value pitch: ", cv2)
        
        #
        self.kit.servo[7].angle = cv1
        self.kit.servo[6].angle = cv2

    def steer_callback(self, data):
        steer = data.steer
        #print("Steer value: {}".format(steer))
        servo_angle = max(0, min((steer * 45 + 45), 90))
        #print("Servo angle: {}".format(servo_angle))
        print("=================================================================")
        self.kit.servo[4].angle = servo_angle 

if __name__ == '__main__':
    try:
        servo_control = Servo_Control()
    except rospy.ROSInterruptException:
        pass