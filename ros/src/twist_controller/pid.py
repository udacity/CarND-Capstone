
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.coeffs = [kp, ki, kd]
        self.min = mn
        self.max = mx

        self.reset()

    def reset(self):
        self.last_error = None
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time
        if self.last_error is not None:
            derivative = (error - self.last_error) / sample_time
        else:
            derivative = 0.0

        val = self.coeffs[0] * error + self.coeffs[1] * integral + self.coeffs[2] * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
