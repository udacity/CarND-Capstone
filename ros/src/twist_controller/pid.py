
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp  = kp
        self.ki  = ki
        self.kd  = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

        self.p  = 0
        self.i  = 0
        self.d  = 0
        self.dt = 0

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral   = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        self.p  = self.kp * error
        self.i  = self.ki * self.int_val
        self.d  = self.kd * derivative
        self.dt = sample_time

        y = self.p + self.i + self.d;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val

    def get_values(self):
        return self.p, self.i, self.d, self.dt