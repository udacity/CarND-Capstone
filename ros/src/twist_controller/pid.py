
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.0

    def reset(self):
        self.int_val = self.last_error = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time

        if sample_time > 1.0e-3:
            derivative = (error - self.last_error) / sample_time
        else:
            derivative = 0.0

        val = self.kp * error + self.ki * self.int_val + self.kd * derivative

        # Take into account actuator limits
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            # Clamping/Anti-wind up, only integrate error if we actuator limits haven't been reached
            self.int_val = integral
        self.last_error = error

        return val