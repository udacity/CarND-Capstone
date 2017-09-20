#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller

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
		#########################
		# Used for the yaw_controller Item
		wheel_base = rospy.get_param('~wheel_base', 2.8498)
		steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
		min_speed = 0.0
		#########################
		self.dbw_en = False


		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

		params = {"wheel_base": wheel_base, "steer_ratio": steer_ratio, "min_speed": min_speed, "max_lat_accel": max_lat_accel, "max_steer_angle": max_steer_angle, "brake_deadband": brake_deadband, "accel_limit": accel_limit, "decel_limit": decel_limit}

		self.controller = Controller(**params)

		rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
		rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

		rate = rospy.Rate(50) # 50Hz
		while not rospy.is_shutdown():
			try:
				self.loop()
				rate.sleep()
			except Exception, e:
				print(e)
				

	def loop(self):

		# EAFP approach to attributes access. The access to self.dbw_en, self.twist_cmd and self.current_velocity will throw an exception if these values were never populated (messages never sent in the topics) 
		try:
			# Execute the controller routine only if the dbw_enable signal is == True
			if(self.dbw_en):
				parameters = {'twist_cmd': self.twist_cmd,
							  'current_vel': self.current_velocity}
				
				throttle, brake, steer = self.controller.control(**parameters)

				#self.publish(throttle, brake, steer)
				self.publish(throttle, brake, steer)
				#self.publish(5.0, 0.0, 0.0)
			# If dbw_enable==False, the car is controlled by the driver and the controller need to be resetted.
			else:
				self.controller.reset()
			
		except AttributeError:
			rospy.logwarn("First messages from topics 'dbw_enabled' and/or 'twist_cmd' and/or 'current_velocity' still missing")
			pass 
			


	def publish(self, throttle, brake, steer):

		tcmd = ThrottleCmd()
		tcmd.enable = True
		tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
		tcmd.pedal_cmd = throttle
		self.throttle_pub.publish(tcmd)

		scmd = SteeringCmd()
		scmd.enable = True
		scmd.steering_wheel_angle_cmd = steer
		self.steer_pub.publish(scmd)

		bcmd = BrakeCmd()
		bcmd.enable = True
		bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
		bcmd.pedal_cmd = brake
		self.brake_pub.publish(bcmd)


	def dbw_enabled_cb(self, dbw_en):
		print("Received dbw command")
		self.dbw_en = dbw_en

	def twist_cmd_cb(self, twist):
		#print("Received twist command")
		self.twist_cmd = twist

	def current_velocity_cb(self, vel):
		#print("Received vel command")
		self.current_velocity = vel

if __name__ == '__main__':
	DBWNode()
