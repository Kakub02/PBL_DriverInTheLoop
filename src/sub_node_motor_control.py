# Imports
import rospy
import RPi.GPIO as GPIO
from motor import Motor
from pid_controller import PIDController
from encoder import Encoder

GPIO.setmode(GPIO.BCM)

# Set GPIO pin numbers for motors
#Motor 1
motor_in1 = 23
motor_in2 = 24
motor_ena = 25
#Motor 2
motor_in3 = 23
motor_in4 = 24
motor_enb = 25

#encoder
encoder_pin = 1


# Set GPIO pin numbers for encoder
#Encoder 1
left_encoder_pin = 17
right_encoder_pin = 27
#Encoder 2
left_encoder_pin = 17
right_encoder_pin = 27

# Set PID parameters
kp=1.0
ki=0.1
kd=0.01

# Set maximum speed in kmph
max_speed = 60

ENA = 26
IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5
ENB = 0

min_speed = -30.0
max_speed = 30.0
min_duty_cycle = 0
max_duty_cycle = 100

class Motor_Control():
    def __init__(self):
        self.encoder = Encoder(27)
        self.PID_motor = PIDController(1, 0.1, 0, 0, 0.8)

        rospy.init_node('motor_controller', anonymous=True)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, self.speed_callback)
        rospy.spin()

    def speed_callback(self, data):
        speed = data.data
        rospy.loginfo("Get Speed : {}".format(speed))

    def calculate_duty_cycle(self, speed):
        
        # 입력 속도를 min_speed와 max_speed 사이로 클리핑합니다.
        speed = abs(max(min(speed, max_speed), min_speed))
        print("abs speed: ", speed)
        set_point_speed_fraction = speed / max_speed
        print("setpoint fraction: ", set_point_speed_fraction)
        real_speed_fraction = self.encoder.getRealSpeed()
        print("real fraction: ", real_speed_fraction)
        duty_cycle = 100 * self.PID_motor.calculate(set_point_speed_fraction, real_speed_fraction)
    
        print("duty cycle : ", duty_cycle)
        return duty_cycle

if __name__ == '__main__':
    try:
        motor_control = Motor_Control()
    except rospy.ROSInterruptException:
        pass