import RPi.GPIO as GPIO
from time import time
#from time import time_ns <- nwm czy dzialo
class Encoder:

    def __init__(self, leftPin, rightPin, callback=None):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        self.state = '00'
        self.direction = None
        self.callback = callback
        self.currentRPM = 0
        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)  
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)  
        
        
    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        newState = "{}{}".format(p1, p2)

        if self.state == "00": # Resting position
            if newState == "01": # Turned right 1
                self.direction = "R"
            elif newState == "10": # Turned left 1
                self.direction = "L"

        elif self.state == "01": # R1 or L3 position
            if newState == "11": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Turned left 1
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10": # R3 or L1
            if newState == "11": # Turned left 1
                self.direction = "L"
            elif newState == "00": # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else: # self.state == "11"
            if newState == "01": # Turned left 1
                self.direction = "L"
            elif newState == "10": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                        
        currentTime = time.time()
        if prevTime != None:
            deltaTime = float((currentTime-prevTime)/1.0e6)
            velocity = (self.value - prevPos)/deltaTime
            # 1920 = 16 impulsow razy przekladnia 120:1, 32,5 = srednica kola, 60 = 60s -> 1min
            self.currentRPM = float(velocity/(1920*32.5)*60.0) #chyba tak powinno byc max rpm to 160
        
        prevPos = self.value
        prevTime = currentTime
        self.state = newState

    # def realSpeed(self):
    #     currentTime = time.time()
    #     #currentTime = time.time_ns() <- nwm czy dzialo
    #     if prevTime != None:
    #         deltaTime = float((currentTime-prevTime)/1.0e6)
    #         velocity = (self.value - prevPos)/deltaTime
    #         # 1920 = 16 impulsow razy przekladnia 120:1, 32,5 = srednica kola, 60 = 60s -> 1min
    #         self.currentRPM = float(velocity/(1920*32.5)*60.0) #chyba tak powinno byc max rpm to 160
        
    #     prevPos = self.value
    #     prevTime = currentTime
        
        
        
    def getRealSpeed(self):
        return self.currentRPM

    def getValue(self):
        return self.value
    
    def getDirection(self):
        return self.direction