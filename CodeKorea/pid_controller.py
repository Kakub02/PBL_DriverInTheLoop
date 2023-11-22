class PIDController:
    def __init__(self, kp, ki, kd, upper_limit, lower_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        #self.previousError = 0
        self.integral_sum = 0
    
    def calculate(self, set_point_value, real_value):
        error = set_point_value - real_value
        self.integral_sum += error
        #output = self.kp * error + self.ki * self.integral_sum

        # calculating control value
        cv = self.kp * (error + (self.kd/self.ki)*(self.integral_sum))
        self.previousError = error
        
        if cv >= self.upper_limit:
            cv = self.upper_limit
            self.integral_sum = self.integral_sum - error
        elif cv <= self.lower_limit:
            cv = self.lower_limit
            self.integral_sum = self.integral_sum - error
        return cv
