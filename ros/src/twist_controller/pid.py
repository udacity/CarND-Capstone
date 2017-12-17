
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.p_effort = 0
        self.i_effort = 0
        self.d_effort = 0

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_error = 0.0
        self.p_effort = 0
        self.i_effort = 0
        self.d_effort = 0

    def step(self, error, sample_time):
        # We want to cap the integral portion by using only a limited number of measurements.
        # However it is unefficient to keep updating a list of measurements. We decide then
        # to estimate the replace it with an estimate of the average using an EMA.
        self.int_val = 0.2 * error * sample_time + 0.8 * self.int_val

        derivative = (error - self.last_error) / sample_time
        # We calculate derivative portion only if we already had at least one measurement since reset
        if any([self.last_error, self.int_val, self.p_effort, self.i_effort, self.d_effort]):
            derivative = 0

        self.last_error = error
        
        self.p_effort = self.kp * error
        self.i_effort = self.ki * self.int_val
        self.d_effort = self.kd * derivative

        y = self.p_effort + self.i_effort + self.d_effort
        val = max(self.min, min(y, self.max))
	
        return val

    def get_PID(self):

        return self.p_effort, self.i_effort, self.d_effort