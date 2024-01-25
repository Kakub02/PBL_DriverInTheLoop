# DC motor test from dr Krauze
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#motor_in1 = 5
#motor_in2 = 6
motor_ena = 26

#GPIO.setup(motor_in1, GPIO.OUT)
#GPIO.setup(motor_in2, GPIO.OUT)
GPIO.setup(motor_ena, GPIO.OUT)

silnik = GPIO.PWM(motor_ena, 1000) #Nowa instancja PWM
wypelnienie = 20 #Wypełnienie sygnału PWM
silnik.start(wypelnienie) #Uruchomienie sygnału PWM
