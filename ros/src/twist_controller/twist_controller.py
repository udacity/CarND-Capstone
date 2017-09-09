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

	def control(self, vel_error, cte, dt):
		brake = 0
		steering = 0

		throttle = self.throttle.step(vel_error, dt)
		steering = self.steering.step(cte, dt)

		return throttle, brake, steering
