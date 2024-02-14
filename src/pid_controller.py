class PID_controller:
    def __init__(self, Kp, Ki, Kd, tau, lower_limit, upper_limit, lower_int_limit, upper_int_limit, T) -> None:
        # Controller gains 
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

   	    # Derivative low-pass filter time constant
        self.tau = tau # 1/2 sekundy

        # Output limits
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        
        # Integrator limits
        self.lower_int_limit = lower_int_limit
        self.upper_int_limit = upper_int_limit
        
    	# Sample time (in seconds)
        self.T = T

        # Controller memory
        self.integrator = 0.0
        self.previous_error = 0.0 # Required for integrator
        self.differentiator = 0.0
        self.previous_measurement = 0.0 # Required for differentiator

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
