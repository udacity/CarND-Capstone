#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher(
            '/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher(
            '/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher(
            '/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        self.dbw_enabled = True
        self.current_linear_velocity = 0
        self.target_linear_velocity = 0
        self.target_angular_veloctiy = 0

        self.controller = Controller(
            vehicle_mass,
            decel_limit,
            accel_limit,
            wheel_radius,
            wheel_base,
            steer_ratio,
            max_lat_accel,
            max_steer_angle
        )

        rospy.Subscriber(
            '/current_velocity', TwistStamped,
            self.current_velocity_cb, queue_size=1)
        rospy.Subscriber(
            '/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber(
            '/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control(
                self.dbw_enabled,
                self.target_linear_velocity,
                self.target_angular_veloctiy,
                self.current_linear_velocity)

            self.publish(throttle, brake, steering)
            rate.sleep()

    def current_velocity_cb(self, curr_velocity):
        self.current_linear_velocity = curr_velocity.twist.linear.x

    def twist_cmd_cb(self, twist_cmd):
        self.target_linear_velocity = twist_cmd.twist.linear.x
        self.target_angular_veloctiy = twist_cmd.twist.angular.z

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

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
