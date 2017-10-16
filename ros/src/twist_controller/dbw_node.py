#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

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


VERBOSE = False       # Print debug logs

KM_TO_MILE = 0.621371
MILE_TO_KM = 1.60934


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        wheel_radius    = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        # Subscriber message data
        self.target_twist     = None
        self.current_velocity = None
        self.dbw_enabled      = False

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback )
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_callback)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():

            if ((self.current_velocity is not None) and (self.target_twist is not None)):

                # Extract the relevant message data from the subscribed topics
                proposed_linear_velocity = self.target_twist.linear.x
                proposed_angular_velocity = self.target_twist.angular.z

                current_linear_velocity = self.current_velocity.linear.x

                # TODO: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
                #                                                     <proposed angular velocity>,
                #                                                     <current linear velocity>,
                #                                                     <dbw status>,
                #                                                     <any other argument you need>)
                throttle, brake, steer = self.controller.control(proposed_linear_velocity,
                                                                proposed_angular_velocity,
                                                                current_linear_velocity,
                                                                self.dbw_enabled)

                # Publish only if driving by wire is enabled
                if self.dbw_enabled:
                    self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):

        throttle_cmd = ThrottleCmd()
        throttle_cmd.enable = True
        throttle_cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        throttle_cmd.pedal_cmd = throttle
        self.throttle_pub.publish(throttle_cmd)

        steering_cmd = SteeringCmd()
        steering_cmd.enable = True
        steering_cmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(steering_cmd)

        brake_cmd = BrakeCmd()
        brake_cmd.enable = True
        brake_cmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        brake_cmd.pedal_cmd = brake
        self.brake_pub.publish(brake_cmd)

    def velocity_callback(self, msg):
        """
        Update current velocity with latest information received.
        """
        self.current_velocity = msg.twist

    def dbw_status_callback(self, msg):
        """
        Update information of whether driving by wire is enabled
        """
        self.dbw_enabled = bool(msg.data)

    def twist_callback(self, msg):
        """
        Update twist with the latest information received.
        """
        self.target_twist = msg.twist

        if VERBOSE:
            self._print_twist(self.twist)

    @staticmethod
    def _print_twist(twist):
        """
        Print twist message for debugging purposes
        """
        t_l__x = twist.linear.x
        t_l__y = twist.linear.y
        t_l__z = twist.linear.z

        t_a__x = twist.angular.x
        t_a__y = twist.angular.y
        t_a__z = twist.angular.z

        rospy.loginfo('LINEAR  x: {} y: {} z: {}'.format(t_l__x, t_l__y, t_l__z))
        rospy.loginfo('ANGULAR x: {} y: {} z: {}'.format(t_a__x, t_a__y, t_a__z))
        rospy.loginfo('--------------------------------------------')


if __name__ == '__main__':
    DBWNode()
