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

	def control(current_velocity, taget_velocity, throttle_pid, brake_pid, steering_pid):
		
		# Break down velocities into linear and angular components
		current_velocity_linear = current_velocity.linear.x
		current_velocity_angular = current_velocity.angular.z
		target_velocity_linear = target_velocity.linear.x
		target_velocity_angular = target_velocity.angular.z

		# Calculate difference between target and current velocity. This will be our CTE for throttle PID. 
		velocity_error = target_velocity_linear - current_velocity_linear

		# Get time using t = 1/F. F Being the frequency in Hz (hardcoded, have to be passed)
		dt = 1 / 10; 



		return throttle, brake, steering