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

import collections as cl
Gains = cl.namedtuple('Gains', 'Kp Ki Kd')  # Data structure for holding PID gains

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

        # throttle_gains = Gains(rospy.get_param('~throttle_Kp', 0.0),
        #                        rospy.get_param('~throttle_Ki', 0.0),
        #                        rospy.get_param('~throttle_Kd', 0.0))
        # steering_gains = Gains(rospy.get_param('~steering_Kp', 0.0),
        #                        rospy.get_param('~steering_Ki', 0.0),
        #                        rospy.get_param('~steering_Kd', 0.0))

        # Parameterize loop rate with 50 Hz as default if not found
        self.rate_param = rospy.get_param('~rate_param', 50)
        
        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Pass params to `Controller` constructor
        self.controller = Controller(wheel_base=wheel_base, steer_ratio=steer_ratio, min_speed=1.0*0.447,
                                     max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle,
                                     accel_limit=accel_limit, decel_limit=decel_limit)

        # Subscriptions
        rospy.Subscriber('/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)

        # Member vars
        self.dbw_enabled = True
        self.current_velocity = None
        self.twist_cmd = None
        self.loop()


    def loop(self):
        rate = rospy.Rate(self.rate_param) # 50Hz means that it goes through the loop 50 times per sec
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if self.twist_cmd is None or self.current_velocity is None:
                continue

            throttle, brake, steering = self.controller.control(self.twist_cmd.twist.linear,
                self.twist_cmd.twist.angular,
                self.current_velocity.twist.linear,
                self.dbw_enabled)

            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        rospy.loginfo("publishing throttle %f, brake %f, steer %f", throttle, brake, steer)
        # We'll either publish the throttle or brake, not both

        if throttle != 0:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
        else:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)


    def dbw_enabled_cb(self, msg):
        rospy.loginfo("DBW status changed to: %s", msg)
        self.dbw_enabled = msg

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def twist_cmd_cb(self, msg):
        rospy.loginfo("Received twist command %s", msg)
        self.twist_cmd = msg



if __name__ == '__main__':
    DBWNode()