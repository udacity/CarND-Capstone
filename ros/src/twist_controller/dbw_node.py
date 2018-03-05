#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller
from twist_controller import PIDControllerProperties, YawControllerProperties, LowPassFilterProperties


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

        # @todo Find usage of vehicle_mass, fuel_capacity
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

        # Publishers (seer, throttle, brake)
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Creates property structs for various controllers
        yaw_controller_properties = YawControllerProperties(wheel_base, steer_ratio, 7.0/3.6, max_lat_accel, max_steer_angle)
        pid_controller_properties = PIDControllerProperties(decel_limit, accel_limit)
        lowpass_filter_properties = LowPassFilterProperties(wheel_radius, brake_deadband)

        # @done: Handover correct parameters gathered from param server
        self.controller = Controller(pid_controller_properties,
                                     yaw_controller_properties,
                                     lowpass_filter_properties)

        # @done: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.is_dbw_enabled = False
        self.current_linear_velocity = 0.0
        self.proposed_linear_velocity = 0.0
        self.proposed_angular_velocity = 0.0

        self.loop()


    def current_velocity_cb(self, velocity_msg):
        self.current_linear_velocity = velocity_msg.twist.linear.x
        rospy.loginfo("Gauss - Got Current Velocity: " + str(self.current_linear_velocity))


    def twist_cmd_cb(self, twist_msg):
        self.proposed_linear_velocity = twist_msg.twist.linear.x
        self.proposed_angular_velocity = twist_msg.twist.angular.z
        rospy.loginfo("Gauss - Got Twist Command (linear, angular): " \
                      + str(self.proposed_linear_velocity) + ", " \
                      + str(self.proposed_angular_velocity))


    def dbw_enabled_cb(self, enabled):
        rospy.loginfo("Gauss - Got DBW Enabled Flag: " + str(enabled))
        self.is_dbw_enabled = enabled


    def loop(self):
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            # @done: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            throttle, brake, steering = self.controller.control(self.proposed_linear_velocity,
                                                                self.proposed_angular_velocity,
                                                                self.current_linear_velocity,
                                                                self.is_dbw_enabled)
            if self.is_dbw_enabled:
                self.publish(throttle, brake, steering)

            rate.sleep()


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


if __name__ == '__main__':
    DBWNode()
