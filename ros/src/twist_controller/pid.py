
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, min=MIN_NUM, max=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = min
        self.max = max

        self.integral = self.last_error = 0.

    def reset(self):
        self.integral = 0.0

    def step(self, error, sample_time):
        integral = self.integral + error * sample_time
        derivative = (error - self.last_error) / sample_time

        result = self.kp * error + self.ki * integral + self.kd * derivative

        if result > self.max:
            result = self.max
        elif result < self.min:
            result = self.min
        else:
            self.integral = integral
        self.last_error = error

        return result
