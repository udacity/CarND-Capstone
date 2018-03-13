#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller_mod import Controller


'''
DBW node for controlling throttle, brake, and steering.
Partially inspired by DataSpeed DBW controller:
https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/ecfb61cbad05e06d5de0d6a971c63c955230a982/README.pdf
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

        # TODO - define min_speed as a parameter in the parameter server?
        min_speed = 0.

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.target_linear_velocity = None
        self.target_angular_velocity = None
        self.current_linear_velocity = None
        self.actual_linear_velocity = 0.0
        self.update_rate_hz = 50.0

        # assume drive by wire is disabled to begin
        self.dbw_enabled = False

        # create low pass filter for measuring acceleration
        default_update_rate = 1.0 / self.update_rate_hz

        # Create `Controller` object
        self.controller = Controller(
            default_update_rate, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, decel_limit, accel_limit, fuel_capacity, vehicle_mass, wheel_radius)

        # Subscribe to messages about car being under drive by wire control
        rospy.Subscriber(
            '/vehicle/dbw_enabled', Bool, self.handleDBWEnabledMessage)

        # Subscribe to messages from waypoint follower
        rospy.Subscriber(
            '/twist_cmd', TwistStamped, self.handle_target_velocity_message)

        # Subscribe to steering report from vehicle
        rospy.Subscriber(
            '/current_velocity', TwistStamped, self.handle_steering_report)

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.update_rate_hz)
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering
            if (self.can_predict_controls()):
                throttle, brake, steering = self.controller.control(self.target_linear_velocity,
                                                                    self.target_angular_velocity,
                                                                    self.actual_linear_velocity)
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

    def handleDBWEnabledMessage(self, dbw_enabled_message):
        self.dbw_enabled = dbw_enabled_message.data
        rospy.loginfo(
            'Drive by Wire %s', "enabled" if self.dbw_enabled else "disabled")
        if (not self.dbw_enabled):
            self.controller.reset()

    def handle_target_velocity_message(self, twist_velocity_command_message):
        twist = twist_velocity_command_message.twist
        self.target_linear_velocity = twist.linear.x
        self.target_angular_velocity = twist.angular.z

    def handle_steering_report(self, twist_current_velocity_message):
        # record actual linear vehicle velocity
        self.actual_linear_velocity = twist_current_velocity_message.twist.linear.x

    def can_predict_controls(self):
        return self.dbw_enabled and self.target_linear_velocity is not None and self.target_angular_velocity is not None and self.actual_linear_velocity is not None

if __name__ == '__main__':
    DBWNode()
