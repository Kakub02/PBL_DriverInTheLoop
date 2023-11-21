import RPi.GPIO as GPIO
import time


class Encoder:
   #encoder1 = Encoder(27, 17, valueChanged)
   
    def __init__(self, pin, callback=None):
        self.pin = pin
        
        self.value = 0
        self.callback = callback
        # self.currentRPM = 0
        self.previousTime = time.time()
        
        self.maxMotorSpeedRPM = 150  # Maximum speed of the motor in RPM
        #----Pololu moze byc 100rpm----#

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self.transitionOccurred)

    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.pin)

        self.value = self.value + 1

    def getRealSpeed(self):
        currentTime = time.time()
        timeElapsed = currentTime - self.previousTime

        if timeElapsed == 0:
            return 0.0

        # Calculate speed in transitions per second
        speedTransactionsPerSecond = self.value / timeElapsed

        # Convert speed to RPM
        pulsesPerRevolution = 24
        currentSpeedRPM = (speedTransactionsPerSecond * 60) / pulsesPerRevolution
        
        fraction = currentSpeedRPM / self.maxMotorSpeedRPM
        self.value = 0
        self.previousTime = currentTime

        return fraction

    def getValue(self):
        return self.value

