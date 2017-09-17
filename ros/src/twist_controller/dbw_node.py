#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import std_msgs.msg

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
	def __init__(self):
		rospy.init_node('dbw_node', log_level=rospy.INFO)

		rospy.loginfo("debug - dbw_node initialised")
		print("dbw_node initialised")

		vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
		fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
		brake_deadband = rospy.get_param('~brake_deadband', .1)
		decel_limit = rospy.get_param('~decel_limit', -5)
		accel_limit = rospy.get_param('~accel_limit', 1.)
		wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
		wheel_base = rospy.get_param('~wheel_base', 2.8498)
		steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

		# Init parameters to control the car 
		#self.dbw_enabled = False  # Coming from /vehicle/dbw_enabled
		rospy.loginfo("debug - Defining variables...")
		self.dbw_enabled = True # Coming from /vehicle/dbw_enabled
		self.target_velocity = None # Commig from /twist_cmd
		self.current_velocity = None # Commig from /current_velocity
		self.current_pose = None # Commig from /current_pose
		self.prev_throttle = 0
		rospy.loginfo("debug - Variables defined")

		# Define PID controller for throttle and brake
		rospy.loginfo("debug - Defining PID...")
		self.throttle_pid = PID(kp=0.1, ki=0.015, kd=0.15, mn=decel_limit, mx=accel_limit)
		self.brake_pid = PID(kp=50.0, ki=0.001, kd=0.15, mn=brake_deadband, mx=1500)
		rospy.loginfo("debug - PID defined")

		# Define Publishers
		rospy.loginfo("debug - Defining publishers...")
		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
		rospy.loginfo("debug - Publishers defined")

		# Instantiate a Controller object
		rospy.loginfo("debug - Creating controler...")
		self.controller = Controller(self.prev_throttle, wheel_base = wheel_base, steer_ratio = steer_ratio,
									min_speed = 0.2, max_lat_accel = max_lat_accel, max_steer_angle = max_steer_angle)
		rospy.loginfo("debug - Controller created")

		# Define Subscribers 
		rospy.loginfo("debug - Defining subscribers...")
		rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb, queue_size=1)
		rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cb)
		rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enable_cb)
		rospy.loginfo("debug - Subscribers defined")

		# Compute and send driving command
		self.loop()


	def loop(self):

		rospy.loginfo("debug - Inside the loop function")

		rate = rospy.Rate(10) # 10Hz
		while not rospy.is_shutdown():

			rospy.loginfo("debug - Inside the while")

			if self.target_velocity is not None and self.current_velocity is not None: 
				rospy.loginfo("debug - Calculate steering, throttle and brake ...")
				# Smoothing out the steering received

				throttle, brake, steering = self.controller.control(
												self.twist.linear.x, 
												self.twist.angular.z, 
												self.current_velocity.linear.x,
												self.dbw_enabled)

				rospy.loginfo("debug - Steering = (%s)", steering)

				self.publish(throttle, brake, steering)

				"""
				target_velocity_angular = self.lpf_steering.filt(self.target_velocity.angular.z)
				steering = self.steering.get_steering(self.target_velocity.linear.x, target_velocity_angular, self.current_velocity.linear.x)
				steering = (math.degrees(steering)) # Converting radians to degree
				rospy.loginfo("debug - Steering = (%s)", steering)
				self.publish(0.2, 0, steering)
				"""


			rate.sleep()

	def publish(self, throttle, brake, steer):
		"""
		All the parameters values should be in the range [0,1] as per Udacity requirements
		"""
		tcmd = ThrottleCmd()
		tcmd.enable = True
		tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
		tcmd.pedal_cmd = throttle
		rospy.loginfo(throttle)
		self.throttle_pub.publish(tcmd)

		scmd = SteeringCmd()
		scmd.enable = True
		scmd.steering_wheel_angle_cmd = steer
		rospy.loginfo(steer)
		self.steer_pub.publish(scmd)

		bcmd = BrakeCmd()
		bcmd.enable = True
		bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
		bcmd.pedal_cmd = brake
		rospy.loginfo(brake)
		self.brake_pub.publish(bcmd)

	def dbw_enable_cb(self, msg): 
		self.dbw_enabled = bool(msg.data)
		if(self.dbw_enabled): 
			self.throttle_pid.reset()
			self.brake_pid.reset()

	def current_velocity_cb(self, msg):
		self.current_velocity = msg.twist

	def twist_cb(self, msg): 
		self.target_velocity = msg.twist 

	def current_pose_cb(self, msg): 
		self.current_pose_cb = msg.pose

if __name__ == '__main__':
	try: 
		DBWNode()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start dbw_node.')