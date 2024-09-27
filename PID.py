class PID:
    def __init__(self, kp: float, ki: float, kd: float, cycle_time_seconds: float, i_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cycle_time_seconds = cycle_time_seconds
        self.i_limit = i_limit
        self.previous_integral: float = 0
        self.previous_error: float = 0

    def update(self, goal: float, current: float):
        error = goal - current
        proportional = error * self.kp
        integral = self.previous_integral + (error * self.ki * self.cycle_time_seconds)
        integral = max(min(integral, self.i_limit), -self.i_limit)
        derivative = (error - self.previous_error) * self.kd / self.cycle_time_seconds
        self.previous_integral = integral
        self.previous_error = error
        return proportional + integral + derivative

    def reset(self):
        self.previous_integral = 0
        self.previous_error = 0
