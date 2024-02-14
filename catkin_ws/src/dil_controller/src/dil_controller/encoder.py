import RPi.GPIO as GPIO

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
