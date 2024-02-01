class PID_controller:
    # Controller gains 
    Kp = 0.0
    Ki = 0.0
    Kd = 0.0

   	# Derivative low-pass filter time constant
    tau = 0.0

    # Output limits
    lower_limit = 0.0
    upper_limit = 0.0

    # Integrator limits
    lower_int_limit = 0.0
    upper_int_limit = 0.0

	# Sample time (in seconds)
    T = 0.0

    # Controller memory
    integrator = 0.0
    previous_error = 0.0 # Required for integrator
    differentiator = 0.0
    previous_measurement = 0.0 # Required for differentiator

    def __init__(self, Kp, Ki, Kd, tau, lower_limit, upper_limit, lower_int_limit, upper_int_limit, T) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.tau = tau # 1/2 sekundy

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        self.lower_int_limit = lower_int_limit
        self.upper_int_limit = upper_int_limit
        
        self.T = T

    # Function for calculating actual control value
    def update(self, setpoint, measurement):
        
        # Error signal
        error = setpoint - measurement
        
        # Proportional
        proportional = self.Kp * error
    
        # Integral
        self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.previous_error)

	    # Anti-wind-up via integrator clamping
        if (self.integrator > self.upper_int_limit): # -100----200/300
            self.integrator = self.upper_int_limit
        elif (self.integrator < self.lower_int_limit):
            self.integrator = self.lower_int_limit

        # Derivative (band-limited differentiator)
        # Note: derivative on measurement, therefore minus sign in front of equation!
        self.differentiator = -(2.0 * self.Kd * (measurement - self.previous_measurement) + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)


        # Compute output and apply limits
        output = proportional + self.integrator + self.differentiator

        if (output > self.upper_limit):
            output = self.upper_limit
        elif (output < self.lower_limit):
            output = self.lower_limit

	    # Store error and measurement for later use
        self.previous_error = error
        self.previous_measurement = measurement

        return output