#!/usr/bin/env
# Imports
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String, Float32, Bool
#from dc_motor import DC_Motor
#from pid_controller import PIDController
#from encoder import Encoder

class DC_Motor:
    # motor1 = Motor(23, 24, 25)
    def __init__(self, in1, in2, en):
        self.in1 = in1
        self.in2 = in2
        self.en = en

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(0)

    #    def declareSetMotor(self, giveSetpoint, giveDirection):
    #        self.setpoint = giveSetpoint
    #        self.setdirection = giveDirection

    def setSpeed(self, setPointSpeed=0, reverse = 0):
        if reverse == 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif reverse == 1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(setPointSpeed)

    def stopMotor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

GPIO.setmode(GPIO.BCM)

# Set GPIO pin numbers for motors
#Motor 1
motor_in1 = 23
motor_in2 = 24
motor_ena = 25
#Motor 2
motor_in3 = 5
motor_in4 = 6
motor_enb = 26


# Set GPIO pin numbers for encoder
#Encoder 1
#left_encoder_pin = 17
#right_encoder_pin = 27
#Encoder 2
#left_encoder_pin = 17
#right_encoder_pin = 27

# Set PID parameters
# kp=1.0
# ki=0.1
# kd=0.01

# Set maximum speed in kmph
max_speed = 60

class Motor_Control(): # TODO main PID loop as in subscriber_node.py
    def __init__(self):
        self.motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
        self.motor2 = DC_Motor(motor_in3, motor_in4, motor_enb)
        self.speed = 0
        self.reverse = 0 #forward

    def communication(self):
        rospy.Subscriber('/our_msg/speed', Float32, self.callback_speed)
        rospy.Subscriber('/our_msg/reverse', Bool, self.callback_reverse)

    def callback_speed(self, msg):
        self.speed = float(msg.data)
        if self.speed > max_speed:
            self.speed = max_speed
        elif self.speed < 0.1:
            self.speed = 0
        print(self.speed)

    def callback_reverse(self, msg):
        self.reverse = bool(msg.data)
        print(self.speed)

    def run(self):
        self.communication()

        loop = rospy.Rate(50.0) # frequency in Hz

        while not rospy.is_shutdown():
        #tutaj będziemy sterować silnikiem
            self.motor1.setSpeed(self.speed, self.reverse)
            self.motor2.setSpeed(self.speed, self.reverse)
            loop.sleep()

if __name__ == "__main__":
    rospy.init_node("Motor_node", anonymous=True)
    motor_control = Motor_Control()
    motor_control.run()
