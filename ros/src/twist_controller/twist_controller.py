
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg   import Float32
#from styx.srv import ChangeControlPIDCoeff


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
	def __init__(self, *args, **kwargs):
		
		# Define the 2 PIDs: one for the throttle/brake control, the second one for the steering
		self.pid_control = PID(3, .5, .125, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
		self.pid_steering = PID(2 ,.0, .0)

		#self.pid_control  = PID(5.0, 0.1, 0.02)
		#self.pid_steering = PID(0.6, 0.7, 0.4)

		# Define the low pass filter to be applied to steering error value
		self.lpf_steer_error = LowPassFilter(0.2, 0.1)

		self.yaw_controller = YawController(kwargs["wheel_base"], kwargs["steer_ratio"], kwargs["min_speed"], kwargs["max_lat_accel"], kwargs["max_steer_angle"])
		self.brake_deadband = kwargs["brake_deadband"]
		self.time = None

		#service = rospy.Service('~change_PID_coeff', ChangeControlPIDCoeff, self.changePIDCoeff)

		
		pass
	#def changePIDCoeff(self, msg):
	#	return ChangeControlPIDCoeffResponse(True)

	def control(self, *args, **kwargs):

		try:
			twist = kwargs['twist_cmd']
			current_velocity = kwargs['current_vel']
		
			# get current/target velocities (linear and angular) from the received messages in the topics 		
			target_lin_vel = twist.twist.linear.x

			target_ang_vel = twist.twist.angular.z
					
			current_lin_vel = current_velocity.twist.linear.x
			current_ang_vel = current_velocity.twist.angular.z
		
			current_time = rospy.get_time()
			if(self.time != None):
				delta_t = current_time - self.time
				
				speed_err = target_lin_vel - current_lin_vel

				# Execute step function for both the pids to get throttle, steering and brake controls
				throttle_brake = self.pid_control.step(speed_err, delta_t)
			
				# set throttle and brake according to the throttle_brake pid output
				throttle = max(0.0,throttle_brake)
				# Consider the brake deadband value
				#brake = max(0, -throttle_brake) + self.brake_deadband
				brake = max(0.0, -throttle_brake)

				if(brake < self.brake_deadband):
					brake = 0.0

				# convert angular speed (current and target) to steering angle (current and target)
				current_steer = self.yaw_controller.get_steering(current_lin_vel, current_ang_vel, current_lin_vel)
				target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, target_lin_vel)
				
				#steer_err = self.lpf_steer_error.filt(target_steer - current_steer)
				
				steer_err = target_steer - current_steer
			
				steer = self.pid_steering.step(steer_err, delta_t)

				#rospy.loginfo('Current PIDs target data:')
				#rospy.loginfo('SpeedCurrent -> %f, SpeedTarget --> %f, SteerCurrent -> %f, SteerTarget --> %f', current_lin_vel, target_lin_vel, steer, target_steer)
				# USED for PID calibration
				#rospy.loginfo('%f', current_lin_vel)

				self.time = current_time

				return throttle, brake, steer
			else:
				self.time = current_time
				return 0.0, 0.0, 0.0
		except Exception, e:
			print(e)
			pass
		#return 1., 0., 0.

	def reset(self):
		self.pid_control.reset()
		self.pid_steering.reset()
