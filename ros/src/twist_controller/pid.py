
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time, log_handle):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;
        
        p_effort = self.kp * error
        i_effort = self.ki * self.int_val
        d_effort = self.kd * derivative

        y = p_effort + i_effort + d_effort
        val = max(self.min, min(y, self.max))
	
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error
        
        self.log_data(log_handle, p_effort, i_effort, d_effort)
   
        return val

    def log_data(self, log_handle, *args):
        log_handle.write(','.join(str(arg) for arg in args) + ',')
