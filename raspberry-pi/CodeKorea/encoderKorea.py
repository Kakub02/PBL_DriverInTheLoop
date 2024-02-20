import RPi.GPIO as GPIO
import time


class Encoder:
    def __init__(self, pin, callback=None):
        self.pin = pin
        
        self.value = 0
        self.callback = callback
        self.maxMotorSpeedRPM = 100  # Maximum speed of the motor in RPM
        self.pulsesPerRevolution = 24

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self.transitionOccurred)

    def transitionOccurred(self, channel):
        self.value += 1

    def getRealSpeed(self):
        currentTime = time.time()
        timeElapsed = currentTime - self.previousTime

        if timeElapsed == 0:
            return 0.0

        # Calculate speed in transitions per second
        speedTransactionsPerSecond = self.value / timeElapsed

        # Convert speed to RPM
        currentSpeedRPM = (speedTransactionsPerSecond * 60) / self.pulsesPerRevolution
        
        fraction = currentSpeedRPM / self.maxMotorSpeedRPM
        self.value = 0
        self.previousTime = currentTime

        return fraction
