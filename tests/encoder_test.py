import RPi.GPIO as GPIO
import time


class Encoder_test:
   #encoder1 = Encoder(27, 17)
   
    def __init__(self, leftPin, rightPin):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        self.state = "00"
        self.direction = None
        self.previousTime = time.time()
        self.maxMotorSpeedRPM = 150  # Maximum speed of the motor in RPM
        self.pulsesPerRevolution = 1920

        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)

    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        newState = "{}{}".format(p1, p2)

        if self.state == "00":  # Resting position
            if newState == "01":  # Turned right 1
                self.direction = "R"
            elif newState == "10":  # Turned left 1
                self.direction = "L"

        elif self.state == "01":  # R1 or L3 position
            if newState == "11":  # Turned right 1
                self.direction = "R"
            elif newState == "00":  # Turned left 1
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10":  # R3 or L1
            if newState == "11":  # Turned left 1
                self.direction = "L"
            elif newState == "00":  # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else:  # self.state == "11"
            if newState == "01":  # Turned left 1
                self.direction = "L"
            elif newState == "10":  # Turned right 1
                self.direction = "R"
            elif newState == "00":  # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        self.state = newState

    def getRealSpeed(self):
        currentTime = time.time()
        timeElapsed = currentTime - self.previousTime

        if timeElapsed == 0:
            return 0.0

        # Calculate speed in transitions per second
        speedTransactionsPerSecond = self.value / timeElapsed

        # Convert speed to RPM
        currentSpeedRPM = (speedTransactionsPerSecond * 60) / self.pulsesPerRevolution

        # Calculate the fraction of the speed relative to the maximum motor speed
        fraction = currentSpeedRPM / self.maxMotorSpeedRPM

        self.value = 0
        self.previousTime = currentTime

        return fraction
