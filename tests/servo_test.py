#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants
nbPCAServo=16 

#Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)

# function init 
def init():

    #for i in range(nbPCAServo):
    pca.servo[0].set_pulse_width_range(MIN_IMP[0] , MAX_IMP[0])


# function main 
#def main():

#   pcaScenario();


# function pcaScenario 
def pcaScenario():
    """Scenario to test servo"""
    #for i in range(nbPCAServo):
    for j in range(MIN_ANG[0],MAX_ANG[0],1):
        print("Send angle {} to Servo {}".format(j,0))
        pca.servo[0].angle = j
        time.sleep(0.01)
    for j in range(MAX_ANG[0],MIN_ANG[0],-1):
        print("Send angle {} to Servo {}".format(j,0))
        pca.servo[0].angle = j
        time.sleep(0.01)
    pca.servo[0].angle=None #disable channel
    time.sleep(0.5)




if __name__ == '__main__':
    init()
    pcaScenario()