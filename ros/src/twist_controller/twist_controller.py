GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Import helper classes
from  yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy


class Controller(object):
	def __init__(self, *args, **kwargs):
		self.throttle = args[0]
		self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'] * 8,
											kwargs['min_speed'], kwargs['max_lat_accel'],
											kwargs['max_steer_angle'])
		self.lpf_steering = LowPassFilter(.96, 1)

	def control(self, *args, **kwargs):
		twist_linear_x = args[0]
		twist_angular_x = args[1]
		current_linear_x = args[2]
		dbw_enabled = args[3]
		
		#if (current_linear_x < twist_linear_x) and self.throttle < 1.0 :
			#self.throttle += 0.1

		# Smooth out value received from simulator
		twist_angular_z = self.lpf_steering.filt(twist_angular_z)
		twist_angular_z = self.lpf_steering.get()
		
		steering = self.yaw_controller.get_steering(twist_linear_x, twist_angular_z, current_linear_x)
		rospy.loginfo("debug - Steering wo lp = (%s)", steering)
		steering = math.degrees(steering)
		rospy.loginfo("debug - Steering w lp = (%s)", steering)

		return 0.2, 0, steering