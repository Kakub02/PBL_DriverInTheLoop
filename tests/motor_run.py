import time
import RPi.GPIO as GPIO
from motor import Motor

GPIO.setmode(GPIO.BCM)

motor1 = Motor(23, 24, 25)

while True:
    try:
        speed_input = int(input("Enter speed (0-100): "))
        direction_input = int(input("Enter direction (-1 = backward/ 1 = forward ): "))
        
        motor1.setMotor(speed_input, direction_input)
    except ValueError:
        print("Invalid input. Please enter a valid integer.")
        continue
    
GPIO.cleanup()