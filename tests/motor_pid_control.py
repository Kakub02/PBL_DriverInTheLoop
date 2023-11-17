#import RPi.GPIO as GPIO
from time import sleep
import math

def control(): #?
    print('')
    #getTargetSpeed()
    #getRealSpeed()
    #calculateError()
    #setSpeed()



if __name__ == '__main__':
    while(1):
        control()

def setMotor(direction, velocity)

def PID_regulator(error):

    # Convert count/s to RPM
    float v1 = velocity1/600.0*60.0
    float v2 = velocity2/600.0*60.0

    v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev
    v1Prev = v1
    v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev
    v2Prev = v2
    kp = 1
    error = float(0) # w przyszlosci error = terget_velocity - filter
    u = kp*error

    direction = 1
    if(u < 0):
        direction = -1
    
    pwr = math.fabs(u) 





