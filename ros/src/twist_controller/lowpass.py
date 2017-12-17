
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.)
        self.val = 0.

    def get(self):
        return self.val

    def reset(self):
        self.val = 0.

    def filt(self, val):
        self.val = self.a * val + self.b * self.val
        return self.val
