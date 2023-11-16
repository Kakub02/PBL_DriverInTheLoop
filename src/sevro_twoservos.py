# Import libraries
import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pins 11 & 12 as outputs, and define as PWM servo1 & servo2
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50) # pin 11 for servo1
GPIO.setup(12,GPIO.OUT)
servo2 = GPIO.PWM(12,50) # pin 12 for servo2

# Start PWM running on both servos, value of 0 (pulse off)
servo1.start(0)
servo2.start(0)

# Turn servo1 to 90
servo1.ChangeDutyCycle(7)
time.sleep(0.5)
servo1.ChangeDutyCycle(0)

# Wait for 2 seconds
time.sleep(2)

# Turn servo2 to 90 & servo1 back to 0
servo2.ChangeDutyCycle(7)
servo1.ChangeDutyCycle(2)
time.sleep(0.5)
servo2.ChangeDutyCycle(0)
servo1.ChangeDutyCycle(0)

# Wait again for 2 seconds! :)
time.sleep(2)

# Turn servo2 to 180 & servo1 to 90
servo2.ChangeDutyCycle(12)
servo1.ChangeDutyCycle(7)
time.sleep(0.5)
servo2.ChangeDutyCycle(0)
servo1.ChangeDutyCycle(0)

# Another little 2 second pause...
time.sleep(2)

# Turn both servos back to 0
servo2.ChangeDutyCycle(2)
servo1.ChangeDutyCycle(2)
time.sleep(0.5)
servo2.ChangeDutyCycle(0)
servo1.ChangeDutyCycle(0)

time.sleep(2)

#Clean things up at the end
servo1.stop()
servo2.stop()
GPIO.cleanup()

print ("Goodbye")