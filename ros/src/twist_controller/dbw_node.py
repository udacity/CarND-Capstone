#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

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

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.target_linear_velocity = None
        self.target_angular_velocity = None
        self.current_linear_velocity = None

        # assume drive by wire is disabled to begin
        self.dbw_enabled = False

        # Create `Controller` object
        self.controller = Controller()

        # Subscribe to messages about car being under drive by wire control
        rospy.Subscriber(
            '/vehicle/dbw_enabled', Bool, self.handleDBWEnabledMessage)

        # Subscribe to messages from waypoint follower
        rospy.Subscriber(
            '/twist_cmd', TwistStamped, self.handle_target_velocity_message)

        # Subscribe to messages from car reporting current velocity
        rospy.Subscriber(
            '/current_velocity', TwistStamped, self.handle_current_velocity_message)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering
            if (self.can_predict_controls()):
                throttle, brake, steering = self.controller.control(self.target_linear_velocity,
                                                                    self.target_angular_velocity,
                                                                    self.current_linear_velocity,
                                                                    self.dbw_enabled)
                if self.dbw_enabled:
                    #   TODO - self.publish(throttle, brake, steer)
                    pass
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

    def handleDBWEnabledMessage(self, dbw_enabled_message):
        self.dbw_enabled = dbw_enabled_message.data
        rospy.loginfo(
            'Drive by Wire %s', "enabled" if self.dbw_enabled else "disabled")

    def handle_target_velocity_message(self, twist_velocity_command_message):
        twist = twist_velocity_command_message.twist
        self.target_linear_velocity = twist.linear.x
        self.target_angular_velocity = twist.angular.z

    def handle_current_velocity_message(self, twist_current_velocity_message):
        self.current_linear_velocity = twist_current_velocity_message.twist.linear.x

    def can_predict_controls(self):
        return self.target_linear_velocity is not None and self.target_angular_velocity is not None and self.current_linear_velocity is not None

if __name__ == '__main__':
    DBWNode()
