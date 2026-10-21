from constants import PID_INTEGRAL_MAX

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        """Crucial for preventing windup when switching from searching to targeting pillars!"""
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        # Anti-windup
        self.integral = max(min(self.integral, PID_INTEGRAL_MAX), -PID_INTEGRAL_MAX)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        return (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)