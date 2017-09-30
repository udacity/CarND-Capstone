
"""
First order Low-pass filter:
https://en.wikipedia.org/wiki/Low-pass_filter
"""


class LowPassFilter(object):
    def __init__(self, time_interval, time_constant):
        self.alpha = time_interval / (time_interval + time_constant)

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def reset(self):
        self.last_val = 0.
        self.ready = False

    def filt(self, val):
        if self.ready:
            val = self.alpha * val + (1. - self.alpha) * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
