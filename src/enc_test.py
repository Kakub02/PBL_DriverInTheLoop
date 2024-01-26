import RPi.GPIO as GPIO
import time

class Encoder_:

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

# if __name__ == "__main__":
#     GPIO.setmode(GPIO.BCM)
#     encoder = Encoder_(16, 12)
#     while True:
#         print(encoder.getValue())
#         encoder.clearValue()
#         time.sleep(0.02)
