GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Import helper classes
from  yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import math


class Controller(object):
	def __init__(self, *args, **kwargs):
		self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'] * 8,
											kwargs['min_speed'] + ONE_MPH, kwargs['max_lat_accel'],
											kwargs['max_steer_angle'])
		self.throttle_pid = PID(kp=0.1, ki=0.015, kd=0.15, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		#self.lpf_steering = LowPassFilter(.8, .9)
		self.min_speed = kwargs['min_speed']
		self.prev_time = None

	def control(self, *args, **kwargs):
		target_velocity_linear_x = args[0]
		target_velocity_angular_z = args[1]
		current_velocity_linear_x = args[2]
		current_velocity_angular_z = args[3]
		dbw_enabled = args[4]
		throttle = 0.0
		brake = 0.0

		# Compute difference between target and current velocity as CTE for throttle. 
		diff_velocity = target_linear_velocity - current_linear_velocity

		current_time = rospy.get_time()
		dt = 0
		if self.prev_time is not None: 
			dt = current_time - self.prev_time
		self.prev_time = current_time
		
		velocity_controller = self.throttle_pid.step(diff_velocity, dt)
		if velocity_controller > 0:
			throttle = velocity_controller
		elif velocity_controller < 0:
			brake = -velocity_controller

		#if (current_velocity_linear_x < target_velocity_linear_x) and self.throttle < 1.0 :
			#self.throttle += 0.1

		# Smooth out value received from simulator
		#target_velocity_angular_z = self.lpf_steering.filt(target_velocity_angular_z)
		#target_velocity_angular_z = self.lpf_steering.get()
		
		steering = self.yaw_controller.get_steering(target_velocity_linear_x, target_velocity_angular_z, current_velocity_linear_x)
		
		#steering = math.degrees(steering)

		return throttle, brake, steering


"""
def control(self, twist_cmd, current_velocity, dbw_enabled):

        if not dbw_enabled:
            self.pid_controller.reset()
            return 0.0, 0.0, 0.0

        target_linear_velocity = twist_cmd.twist.linear.x
        target_angular_velocity = twist_cmd.twist.angular.z
        current_linear_velocity = current_velocity.twist.linear.x
        current_agular_velocity = current_velocity.twist.angular.z

        #rospy.loginfo("tl = %f, cl = %f, ta = %f", target_linear_velocity, current_linear_velocity, target_angular_velocity)

        linear_velocity_error = target_linear_velocity - current_linear_velocity
        time = rospy.get_time()
        delta_t = time - self.prev_time if self.prev_time is not None else 0
        self.prev_time = time
        linear_control = self.pid_controller.step(linear_velocity_error, delta_t)

        throttle = 0.0
        if linear_control > 0.0:
            throttle = linear_control

        brake = 0.0
        if linear_control < 0.0:
            rospy.loginfo("BRAKE = %f", linear_control)
            brake = -linear_control

        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steer
"""