GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Import helper classes
from  yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID


class Controller(object):
	def __init__(self, throttle, brake, steering):
		"""
		:param throttle - PID controller applied to the throttle
		:param brake - PID controller applied to the brake
		:param steering - PID controller applied to the steering   
		"""
		self.throttle = throttle
		self.brake = brake 
		self.steering = steering

		# Apply a LowPassFilter to smooth out values
		self.throtlle_low_pass = LowPassFilter(.5)
		self.brake_low_pass = LowPassFilter(.5)
		self.steering_low_pass = LowPassFilter(.5)

	def control(self, vel_error, cte, dt):
		brake = 0
		throttle = self.throtlle_low_pass.filt(self.throttle.step(vel_error, dt))
		steering = self.steering.step(cte, dt)

		if throttle > 0:
			self.brake_low_pass.filt(0)
			self.brake.reset()
		else:
			throttle = 0
			self.throttle.reset()
			brake = self.brake.step(-vel_error, dt)
			brake = self.brake_low_pass.filt(brake)

		return throttle, brake, steering