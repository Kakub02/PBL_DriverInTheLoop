#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO

# from servo_control.msg import ServoPosition
# from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import String, Float32, Bool

# from sensor_msgs.msg import Imu
# import RPi.GPIO as GPIO
# from time import sleep
# from pid_controller import PIDController
# from mpu6050 import mpu6050

min_roll_pwm = 5
max_roll_pwm = 10

min_pitch_pwm = 5
max_pitch_pwm = 8

min_steer_pwm = 5  # ????????
max_steer_pwm = 8  # ????????

# Set GPIO pin numbers for servos
# Servo roll (boki)
roll_servo_pin = 17
# Servo pitch
pitch_servo_pin = 27
# Servo steer
# steer_servo_pin = ?


def angle_to_duty_cycle(angle):
    return float((angle + 90) / 18 + 2.5) # not sure if the pwm range is 2.5 - 12.5 or 2 - 12 ?????????


class Servo:
    # servo = Servo(23)
    def __init__(self, pwm_pin):
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 50)
        self.pwm.start(0)

    def __del__(self):
        self.pwm.stop()
        GPIO.cleanup() # not sure if it should be here

    def set_duty_cycle(self, duty_cycle):
        self.pwm.ChangeDutyCycle(duty_cycle)


class Servo_Control:
    def __init__(self):
        # self.mpu = mpu6050(0x68)
        # self.PID_servo1 = PIDController(1, 0.1, 0.01)
        # self.PID_servo2 = PIDController(1, 0.1, 0.01)
        self.roll_servo = Servo(roll_servo_pin)
        self.pitch_servo = Servo(pitch_servo_pin)
        # self.steer_servo = Servo(steer_servo_pin)
        self.roll_pwm = 10.5
        self.pitch_pwm = 10.5
        self.steer_pwm = 7.5

    def communication(self):
        rospy.Subscriber("/our_msg/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("/our_msg/roll", Float32, self.roll_callback)
        # rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd_manual", CarlaEgoVehicleControl, self.steer_callback)

    # def imu_callback(self, data):
    #     #'imu' 메시지에서 'orientation' 필드만 출력
    #     orientation = data.orientation
    #     print("IMU orientation: x={}".format(orientation.x))
    #     print("IMU orientation: y={}".format(orientation.y))

    #     #servo1_angle = max(0, min((orientation.x * 250 + 180) / 2, 180))

    #     servo1_angle = (orientation.x * 250 + 180) / 2
    #     if servo1_angle >= upper_limit:
    #         servo1_angle = upper_limit
    #     elif servo1_angle <= lower_limit:
    #         servo1_angle = lower_limit
    #     #servo2_angle = max(0, min((orientation.y * 400 + 180) / 2, 180))
    #     servo2_angle = (orientation.y * 400 + 180) / 2
    #     if servo2_angle >= upper_limit:
    #         servo2_angle = upper_limit
    #     elif servo2_angle <= lower_limit:
    #         servo2_angle = lower_limit

    #     print("roll : ", servo1_angle)
    #     print("pitch : ", servo2_angle)

    #     # get actual gyroscope data
    #     gyro_data = self.mpu.get_gyro_data()
    #     X_mpu = gyro_data['x']
    #     Y_mpu = gyro_data['y']

    #     print("gyro x: ", X_mpu)
    #     print("gyro y: ", Y_mpu)

    #     # use PI control
    #     cv1 = self.PID_servo1.calculate(servo1_angle, X_mpu) # roll
    #     cv2 = self.PID_servo2.calculate(servo2_angle, Y_mpu) # pitch

    #     print("PID calculated value roll", cv1)
    #     print("PID calculated value pitch: ", cv2)

    #     #
    #     self.kit.servo[7].angle = cv1
    #     self.kit.servo[6].angle = cv2

    def pitch_callback(self, msg):
        pitch_angle = float(msg.data)
        self.pitch_pwm = angle_to_duty_cycle(pitch_angle)
        if self.pitch_pwm > max_pitch_pwm:
            self.pitch_pwm = max_pitch_pwm
        elif self.pitch_pwm < min_pitch_pwm:
            self.pitch_pwm = min_pitch_pwm
        print("pitch: ", self.pitch_pwm)

    def roll_callback(self, msg):
        roll_angle = float(msg.data)
        self.roll_pwm = angle_to_duty_cycle(roll_angle)
        if self.roll_pwm > max_roll_pwm:
            self.roll_pwm = max_roll_pwm
        elif self.roll_pwm < min_roll_pwm:
            self.roll_pwm = min_roll_pwm
        print("roll: ", self.roll_pwm)

    # def steer_callback(self, msg):
    #     steer_angle = float(msg.data)
    #     self.steer_pwm = angle_to_duty_cycle(steer_angle)
    #     if self.steer_pwm > max_steer_pwm:
    #         self.steer_pwm = max_steer_pwm
    #     elif self.steer_pwm < min_steer_pwm:
    #         self.roll_pwm = min_steer_pwm
    #     print("roll: ", self.steer_pwm)

    def run(self):
        self.communication()

        loop = rospy.Rate(50.0)  # frequency in Hz

        while not rospy.is_shutdown():
            self.pitch_servo.set_duty_cycle(self.pitch_pwm)
            self.roll_servo.set_duty_cycle(self.roll_pwm)
            # self.steer_servo.set_duty_cycle(self.steer_pwm)
            loop.sleep()


if __name__ == "__main__":
    rospy.init_node("imu_steer_orientation_subscriber", anonymous=True)
    servo_control = Servo_Control()
    servo_control.run()
