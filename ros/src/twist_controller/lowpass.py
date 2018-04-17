
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val


class SimpleFilter(object):
    def __init__(self, k):
        self.k = k
        self.last_val = None

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.last_val:
            val = self.k * self.last_val + (1-self.k) * val
        self.last_val = val
        return val


