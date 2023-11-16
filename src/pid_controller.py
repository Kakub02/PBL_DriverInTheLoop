class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previousError = 0
        self.integral = 0
    
    def calculate(self, setPointSpeed, realSpeed):
        error = setPointSpeed - realSpeed
        self.integral += error
        derivative = error - self.previousError
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previousError = error
        return output