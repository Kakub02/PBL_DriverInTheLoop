import RPi.GPIO as GPIO          
from time import sleep

in1 = 24
in2 = 23
en = 25
temp1 = 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
p = GPIO.PWM(en, 1000)
p.start(0)
print("\n")
print("The default speed & direction of motor is 0 & Forward.....")
print("Input a number between 0 and 100 to set the motor speed.")
print("Type 'e' to exit.")
print("\n")    

while True:
    try:
        speed_input = int(input("Enter speed (0-100): "))
        
        if 0 <= speed_input <= 100:
            duty_cycle = speed_input
            p.ChangeDutyCycle(speed_input)
        elif speed_input == 'r':
            if(temp1 == 1):
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                print("forward")
            else:
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.HIGH)
                print("backward")
        elif speed_input == 'e':
            break

        else:
            print("<<<  wrong data  >>>")
            print("please enter the defined data to continue.....")
    except ValueError:
        print("Invalid input. Please enter a valid integer.")
        continue

GPIO.cleanup()