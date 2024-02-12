#!/usr/bin/env

import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String, Float32, Bool

# from dil_controller_src.encoder import Encoder
# from dil_controller_src.dc_motor import DC_motor
# from dil_controller_src.pid_controller import PID_controller


# Set GPIO pin numbers for motors
# Motor 1
motor_in1 = 22
motor_in2 = 27
motor_ena = 17
# Motor 2
motor_in3 = 23
motor_in4 = 24
motor_enb = 25

# Set GPIO pin numbers for encoder
left_encoder_pin = 16
right_encoder_pin = 12

# Set PID parameters
Kp = 0.000136
Ki = 7.0263
Kd = 0


# Set border values
min_speed_kmh = 0.1
max_speed_kmh = 60

# Border pwm values
min_pwm = 0
max_pwm = 100

# v max of encoder in impulses/second
max_encoder_speed = 10000 # on frequency 50Hz

# frequency in Hz
frequency = 50

# map speed from kilometers/hour to impulses/second
def speed_kmh_to_imps(speed):
    return speed / max_speed_kmh * max_encoder_speed

class DC_Motor:
    def __init__(self, in1, in2, en):
        self.in1 = in1
        self.in2 = in2
        self.en = en

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(0)

    def __del__(self):
        self.stop_motor()
        self.pwm.stop()

    def set_speed(self, setPointSpeed=0, reverse=0):
        if reverse == 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif reverse == 1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(setPointSpeed)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

class Encoder:

    def __init__(self, leftPin, rightPin):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)

    def transitionOccurred(self, channel):
        self.value = self.value + 1

    def getValue(self):
        return self.value

    def clearValue(self):
        self.value = 0

class PID_controller:
    def __init__(self, Kp, Ki, Kd, tau, lower_limit, upper_limit, lower_int_limit, upper_int_limit, T) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.tau = tau # 1/2 sekundy

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        self.lower_int_limit = lower_int_limit
        self.upper_int_limit = upper_int_limit
        
        self.T = T

    # Function for calculating actual control value
    def update(self, setpoint, measurement):
        
        # Error signal
        error = setpoint - measurement
        
        # Proportional
        proportional = self.Kp * error
    
        # Integral
        self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.previous_error)

	    # Anti-wind-up via integrator clamping
        if (self.integrator > self.upper_int_limit): # -100----200/300
            self.integrator = self.upper_int_limit
        elif (self.integrator < self.lower_int_limit):
            self.integrator = self.lower_int_limit

        # Derivative (band-limited differentiator)
        # Note: derivative on measurement, therefore minus sign in front of equation!
        self.differentiator = -(2.0 * self.Kd * (measurement - self.previous_measurement) + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)


        # Compute output and apply limits
        output = proportional + self.integrator + self.differentiator

        if (output > self.upper_limit):
            output = self.upper_limit
        elif (output < self.lower_limit):
            output = self.lower_limit

	    # Store error and measurement for later use
        self.previous_error = error
        self.previous_measurement = measurement

        return output

class Motor_Control:
    def __init__(self):
        self.motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
        self.motor2 = DC_Motor(motor_in3, motor_in4, motor_enb)
        self.encoder = Encoder(left_encoder_pin,right_encoder_pin)
        self.pid = PID_controller(0.1538, 2.263, 0, 1, 0, 100, 0, 250, 1/frequency)

        self.setpoint_speed = 0
        self.reverse = 0  # forward

    def communication(self):
        rospy.Subscriber("/our_msg/speed", Float32, self.speed_callback)
        rospy.Subscriber("/our_msg/reverse", Bool, self.reverse_callback)

    def speed_callback(self, msg):
        speed_kmh = float(msg.data)
        if speed_kmh > max_speed_kmh:
            speed_kmh = max_speed_kmh
        elif speed_kmh < min_speed_kmh:
            speed_kmh = 0
        self.setpoint_speed = speed_kmh_to_imps(speed_kmh)
        print(self.setpoint_speed)

    def reverse_callback(self, msg):
        self.reverse = bool(msg.data)
        print(self.reverse)

    def run(self):
        self.communication()

        loop = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            start_time = time.time()

            encoder_value = self.encoder.getValue()
            self.encoder.clearValue()

            real_speed = encoder_value * frequency
            pid_pwm = self.pid.update(self.setpoint_speed, real_speed)

            self.motor1.set_speed(pid_pwm, self.reverse)
            self.motor2.set_speed(pid_pwm, self.reverse)
            
            loop.sleep()

    def stop(self):
        self.motor1.stop_motor()
        self.motor2.stop_motor()


if __name__ == "__main__":
    rospy.init_node("Motor_node", anonymous=True)
    motor_control = Motor_Control()
    try:
        motor_control.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_control.stop()
        GPIO.cleanup()
