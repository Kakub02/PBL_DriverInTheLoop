import RPi.GPIO as GPIO

# Define GPIO pins for motor control
Motor_EN = 25  # Enable pin kuuuuurwa które tu piny xd
Motor_IN1 = 23  # Input 1
Motor_IN2 = 24  # Input 2

# PWM setup
pwm_range = 100  # PWM range (e.g., 0 to 100)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor_EN, GPIO.OUT)
GPIO.setup(Motor_IN1, GPIO.OUT)
GPIO.setup(Motor_IN2, GPIO.OUT)

# Initialize PWM
pwm = GPIO.PWM(Motor_EN, pwm_range)
pwm.start(0)

def set_motor_speed(pwm_value, reverse):
    if (reverse == 0):  #Do przodu  # nie wiem jeszcze jak bedzie przesylana flaga o kierunku jazdy
        GPIO.output(Motor_IN1, GPIO.HIGH)
        GPIO.output(Motor_IN2, GPIO.LOW)
    elif reverse == 1:  #Do tyłu
        GPIO.output(Motor_IN1, GPIO.LOW)
        GPIO.output(Motor_IN2, GPIO.HIGH)
    else:   #Stop
        GPIO.output(Motor_IN1, GPIO.LOW)
        GPIO.output(Motor_IN2, GPIO.LOW)
    
    pwm.ChangeDutyCycle(pwm_value)

def cleanup():
    GPIO.cleanup()