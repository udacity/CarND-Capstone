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

        self.dbw_enabled = False
        self.current_velocity = None
        self.twist_cmd = None
        self.time_last = None
        self.last_action = ''

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        params = {
            'vehicle_mass': vehicle_mass,
            'fuel_capacity': fuel_capacity,
            'brake_deadband': brake_deadband,
            'decel_limit': decel_limit,
            'accel_limit': accel_limit,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'steer_ratio': steer_ratio,
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle
        }
        self.controller = Controller(**params)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool,
                         self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped,
                         self.twist_cmd_cb, queue_size=1)

        self.loop()

    def are_messages_valid(self):
        """
        Check is subscribed messages are available
        """
        return self.dbw_enabled and \
            self.current_velocity is not None and \
            self.twist_cmd is not None and \
            self.time_last is not None

    def loop(self):
        """
        Get predicted throttle, brake, and steering using `twist_controller`
        """
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            time_now = rospy.get_time()

            if self.are_messages_valid():
                rospy.loginfo('DBW: linear current velocity: %f',
                              self.current_velocity.twist.linear.x)
                rospy.loginfo('DBW: linear setpoint velocity: %f',
                              self.twist_cmd.twist.linear.x)
                delta_t = time_now - self.time_last
                throttle, brake, steer = self.controller.control(
                    current_velocity=self.current_velocity.twist.linear.x,
                    linear_setpoint=self.twist_cmd.twist.linear.x,
                    angular_setpoint=self.twist_cmd.twist.angular.z,
                    delta_t=delta_t)

                rospy.loginfo(
                    'DBW: throttle: %f brake: %f steer: %f', throttle, brake, steer)
                self.publish(throttle, brake, steer)
            else:
                rospy.loginfo('DBW: reset PID controller')
                self.controller.reset()

            self.time_last = time_now
            rate.sleep()

    def publish(self, throttle, brake, steer):
        """
        Publish car control commands
        """
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

        # NOTE(slack): do not publish throttle and brake at the same time,
        # unless we switch from one to the other (to prevent the simulator
        # from keeping the last value)
        action = 'brake' if brake > 0.0 else 'throttle'

        if action != self.last_action:
            self.brake_pub.publish(bcmd)
            self.throttle_pub.publish(tcmd)

        elif action == 'brake':
            self.brake_pub.publish(bcmd)

        elif action == 'throttle':
            self.throttle_pub.publish(tcmd)

        self.steer_pub.publish(scmd)

        self.last_action = action

    def dbw_enabled_cb(self, msg):
        """
        Callback, is drive by wire enabled
        :param msg bool dbw enabled
        """
        rospy.loginfo('DBW: dbw_enabled_cb')
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        """
        :param msg TwistStamped of current velocity
        """
        self.current_velocity = msg

    def twist_cmd_cb(self, msg):
        """
        :param msg TwistStamped of twist command, contains desired 
        linear and angular velocity
        """
        self.twist_cmd = msg


if __name__ == '__main__':
    DBWNode()
