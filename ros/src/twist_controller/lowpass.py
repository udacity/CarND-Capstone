
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.)

        self.last_val = 0.

        # Initialize from last_val = 0 instead of allowing an initial step
        # change on activation when dbw_enabled turns ON
        self.ready = True

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
