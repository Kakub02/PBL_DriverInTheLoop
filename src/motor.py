import RPi.GPIO as GPIO

class Motor:
    # motor1 = Motor(23, 24, 25)
    def __init__(self, in1, in2, en, callback=None):
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

    def setMotor(self, setpoint = 0, setdirection = 1):
        if setdirection == 1:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif setdirection == -1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(setpoint)
            
    def stopMotor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
