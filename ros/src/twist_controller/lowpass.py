class LowPassFilter(object):
	def __init__(self, window_weight):
		self.last_val = 0.
		self.window_weight = window_weight

	def filt(self, value):
		return (self.window_weight * self.last_val) + ((1.0 - self.window_weight) * value)