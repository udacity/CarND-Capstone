#!/usr/bin/env python
"""
A node to control the vehicle's throttle, brake, and steering based on Twist message commands.

This node subscribes to `/twist_cmd` message which provides the proposed linear and angular
velocities.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
"""

from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import rospy
from std_msgs.msg import Bool
from styx_msgs.msg import Lane, Waypoint
from twist_controller import Controller


class DBWNode(object):
    """A brake, throttle, and steering control node implemented as a Python class."""

    def cmd_vel_cb(self, cmd_vel):
        """
        Callback for TwistStamped velocity commands.

        Stores linear and angular velocity commands for later use.
        """
        self.cmd_lin_vel = cmd_vel.twist.linear.x
        self.cmd_ang_vel = cmd_vel.twist.angular.z

    def cur_vel_cb(self, cur_vel):
        """
        Callback for TwistStamped velocity feedback.

        Stores linear and angular velocity feedback for later use.
        """
        self.cur_lin_vel = cur_vel.twist.linear.x
        self.cur_ang_vel = cur_vel.twist.angular.z

    def dbw_enabled_cb(self, dbw_enabled):
        """
        Callback for Bool DBW state messages.

        Stores DBW enabled state for later use.
        """
        self.dbw_enabled = dbw_enabled

    def __init__(self):
        """
        Constructor.

        - Sets up class variables
        - Gets ROS parameters:
          - ~kp_vel
          - ~ki_throttle
          - ~kp_throttle
          - ~min_speed
          - ~vehicle_mass
          - ~fuel_capacity
          - ~brake_deadband
          - ~decel_limit
          - ~accel_limit
          - ~wheel_radius
          - ~wheel_base
          - ~steer_ratio
          - ~max_lat_accel
          - ~max_steer_angle
          - ~rate
        - Subscribes to twist_cmd, current_velocity, and dbw_enabled
        - Advertises steering_cmd, throttle_cmd, and brake_cmd
        """
        self.cmd_lin_vel = 0
        self.cur_lin_vel = 0
        self.cmd_ang_vel = 0
        self.cur_ang_vel = 0
        self.dbw_enabled = True

        # Controller Gains
        kp_vel = rospy.get_param('~kp_vel')
        ki_throttle = rospy.get_param('~ki_throttle')
        kp_throttle = rospy.get_param('~kp_throttle')

        min_speed = rospy.get_param('~min_speed')
        self.rate = rospy.get_param('~rate', 20)

        # Vehicle Parameters
        vehicle_mass = rospy.get_param('~vehicle_mass')
        fuel_capacity = rospy.get_param('~fuel_capacity')
        brake_deadband = rospy.get_param('~brake_deadband')
        decel_limit = rospy.get_param('~decel_limit')
        accel_limit = rospy.get_param('~accel_limit')
        wheel_radius = rospy.get_param('~wheel_radius')
        wheel_base = rospy.get_param('~wheel_base')
        steer_ratio = rospy.get_param('~steer_ratio')
        max_lat_accel = rospy.get_param('~max_lat_accel')
        max_steer_angle = rospy.get_param('~max_steer_angle')

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller = Controller(kp_vel, kp_throttle, ki_throttle, min_speed,
                                     vehicle_mass, fuel_capacity, brake_deadband,
                                     decel_limit, accel_limit,
                                     wheel_radius, wheel_base,
                                     steer_ratio, max_lat_accel, max_steer_angle, self.rate)

        self.cmd_vel_sub = rospy.Subscriber("/twist_cmd", TwistStamped, self.cmd_vel_cb)
        self.cur_vel_sub = rospy.Subscriber("/current_velocity", TwistStamped, self.cur_vel_cb)
        self.dbw_enabled_sub = rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_cb)

    def loop(self):
        """
        Main loop function.

        This function contains an infinite 50 Hz loop that updates the control and calls publish()
        """
        rate = rospy.Rate(self.rate)
        last_time = rospy.get_time()
        while not rospy.is_shutdown():
            curr_time = rospy.get_time()
            dt = curr_time-last_time
            throttle, brake, steering = self.controller.control(self.cmd_lin_vel,
                                                                self.cmd_ang_vel,
                                                                self.cur_lin_vel,
                                                                self.cur_ang_vel,
                                                                dt,
                                                                self.dbw_enabled)
            last_time = curr_time
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        """Publish throttle, brake, and steering."""
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
    rospy.init_node('dbw_node')
    dbw = DBWNode()
    dbw.loop()
