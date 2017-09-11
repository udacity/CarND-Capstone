#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import styx_msgs.msg
import std_msgs.msg

from twist_controller import Controller
from pid import PID
import dbw_utils

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
		rospy.init_node('dbw_node')

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
		self.dbw_enabled = False  # Coming from /vehicle/dbw_enabled
		self.twist_command = None # Commig from /twist_cmd
		self.current_velocity = None # Commig from /current_velocity
		self.final_waypoints = None # Commig from /final_waypoints
		self.current_pose = None # Commig from /current_pose
		self.init_time = rospy.get_rostime()

		# Define PID controller for throttle. brake and steering
		self.throttle_pid = PID(kp=0.1, ki=0.015, kd=0.15, mn=decel_limit, mx=accel_limit)
		self.brake_pid = PID(kp=50.0, ki=0.001, kd=0.15, mn=brake_deadband, mx=1500)
		self.steering_pid = PID(kp=1.0, ki=0.001, kd=0.5, mn=-max_steer_angle, mx=max_steer_angle)

		# Define Publishers
		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

		# Instantiate a Controller object
		self.controller = Controller(self.throttle_pid, self.brake_pid, self.steering_pid)

		# Define Subscribers 
		rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb)
		rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cb)
		rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enable_cb)
		rospy.Subscriber("/current_pose", PoseStamped, self.current_pose_cb, queue_size=1)
		rospy.Subscriber("/final_waypoints", styx_msgs.msg.Lane, self.final_waypoints_cb)

		self.loop()

	def loop(self):
		rate = rospy.Rate(10) # 50Hz
		while not rospy.is_shutdown():

			#if self.dbw_enabled and self.final_waypoints is not None:
			if True: 

				rospy.loginfo("Into the loop")

				crt_time = rospy.get_rostime()

				# Get delta_t in sec. 
				dt = crt_time - self.init_time
				dt = dt.secs + (1e-9 * dt.nsecs)

				# Reset time within the loop (so that we only compute the time of one loop iteration)
				self.init_time = crt_time
				
				# Get linear velocity and CTE. Velocity is the difference between current and desired speed in the future. 
				velocity = self.final_waypoints[1].twist.twist.linear.x - self.current_velocity.linear.x
				cte = dbw_utils.get_cte(self.final_waypoints, self.current_pose_cb)

				# Finally, compute throttle, brake and steer angle to use
				throttle, brake, steering = self.controller.control(velocity, cte, dt)

				self.publish(throttle, brake, steering)
			rate.sleep()

	def publish(self, throttle, brake, steer):
		"""
		All the parameters values should be in the range [0,1] as per Undacity requirements
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
			self.steering_pid.reset()
			self.throttle_pid.reset()
			self.brake_pid.reset()

	def current_velocity_cb(self, msg):
		self.current_velocity = msg.twist

	def twist_cb(self, msg): 
		self.twist_cmd = msg.twist 

	def final_waypoints_cb(self, msg): 
		self.final_waypoints = msg.waypoints

	def current_pose_cb(self, msg): 
		self.current_pose_cb = msg.pose

if __name__ == '__main__':
	DBWNode()