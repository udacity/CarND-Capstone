
MIN_NUM = float('-inf')
MAX_NUM = float('inf')

class PID(object):
    """Generic PID controller class"""

    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        """Constructor for PID class

        Args:
            kp (float): Propotional gain Kp
            ki (float): Integral gain Ki
            kd (float): Derivative gain Kd
            mn (float): Minimum value. Defaults to -inf
            mx (float): Maximum value. Defaults to inf
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        """Reset error (in case of dbw is disabled)"""

        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
