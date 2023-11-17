# Imports
import rospy
import RPi.GPIO as GPIO
from motor import Motor
from pid_controller import PIDController
from encoder import Encoder

# Set GPIO pin numbers for motor
motor_in1 = 23
motor_in2 = 24
motor_en = 25

# Set GPIO pin numbers for encoder
left_encoder_pin = 17
right_encoder_pin = 18

# Set PID parameters
kp=1.0
ki=0.1
kd=0.01

# Set maximum speed in kmph
max_speed = 60

class Motor_Control(): # TODO main PID loop as in subscriber_node.py

    def communication(self):
            # Innit Sub
        rospy.Subscriber('/our_msg/speed', Float32, self.callback_speed)
        rospy.Subscriber('/our_msg/reverse', Bool, self.callback_reverse)

    def callback_speed(msg):
        speed = float(msg.data)

    def callback_reverse(msg):
        reverse = bool(msg.data)

    def run(self):
        self.communication()

        loop = rospy.Rate(10.0) # frequency in Hz

        while not rospy.is_shutdown():
        #tutaj będziemy sterować silnikiem
                
            loop.sleep()


if __name__ == "__main__":
    rospy.init_node("Motor_node", anonymous=True)
    motor_Control = Motor_Control()
    motor_Control.run()
