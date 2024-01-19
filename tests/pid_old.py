

# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor, PID

# Setting general parameters
tstop = 2  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
wsp = 20  # Motor speed set point (rad/s)
tau = 0.1  # Speed low-pass filter response time (s)

# Creating PID controller object
kp = 0.15
ki = 0.35
kd = 0.01
taupid=0.01
pid = PID(tsample, kp, ki, kd, umin=0, tau=taupid)

# Creating motor object using GPIO pins 16, 17, and 18
# (using SN754410 quadruple half-H driver chip)
# Integrated encoder on GPIO pins 24 and 25.
mymotor = Motor(
    enable1=16, pwm1=17, pwm2=18,
    encoder1=24, encoder2=25, encoderppr=300.8)
mymotor.reset_angle()

# Initializing previous and current values
ucurr = 0  # x[n] (step input)
wfprev = 0  # y[n-1]
wfcurr = 0  # y[n]

# Initializing variables and starting clock
thetaprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Running execution loop
print('Running code for', tstop, 'seconds ...')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting motor shaft angular position: I/O (data in)
    thetacurr = mymotor.get_angle()
    # Calculating motor speed (rad/s)
    wcurr = np.pi/180 * (thetacurr-thetaprev)/(tcurr-tprev)
    # Filtering motor speed signal
    wfcurr = tau/(tau+tsample)*wfprev + tsample/(tau+tsample)*wcurr
    wfprev = wfcurr
    # Calculating closed-loop output
    ucurr = pid.control(wsp, wfcurr)
    # Assigning motor output: I/O (data out)
    mymotor.set_output(ucurr)
    # Updating previous values
    thetaprev = thetacurr
    tprev = tcurr

print('Done.')
# Stopping motor and releasing GPIO pins
mymotor.set_output(0, brake=True)
del mymotor