import constants as const

class PID(object):
    def __init__(self, kp, ki, kd, mn=const.MIN_NUM, mx=const.MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.error = {
                "kp_error": 0.0,
                "ki_error": 0.0,
                "kd_error": 0.0
            }

    def reset(self):
        for e in self.error:
            self.error[e]=0.0

    def step(self, error, sample_time):
        self.error["kd_error"] = (error - self.error["kp_error"]) / sample_time
        self.error["kp_error"] =  error
        self.error["ki_error"] += error * sample_time

        val =    self.kp * self.error["kp_error"] + \
                 self.ki * self.error["ki_error"] + \
                 self.kd * self.error["kd_error"]

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min

        return val
