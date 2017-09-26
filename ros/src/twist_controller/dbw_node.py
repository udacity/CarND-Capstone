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

magnitude = lambda x,y,z : math.sqrt(x*x+y*y+z*z)

SAMPLE_RATE = 10  # sample rate in hertz

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.params = {}

        self.params['vehicle_mass'] = rospy.get_param('~vehicle_mass', 1736.35)
        self.params['fuel_capacity'] = rospy.get_param('~fuel_capacity', 13.5)
        self.params['brake_deadband'] = rospy.get_param('~brake_deadband', .1)
        self.params['decel_limit'] = rospy.get_param('~decel_limit', -5)
        self.params['accel_limit'] = rospy.get_param('~accel_limit', 1.)
        self.params['wheel_radius'] = rospy.get_param('~wheel_radius', 0.2413)
        self.params['wheel_base'] = rospy.get_param('~wheel_base', 2.8498)
        self.params['steer_ratio'] = rospy.get_param('~steer_ratio', 14.8)
        self.params['max_lat_accel'] = rospy.get_param('~max_lat_accel', 3.)
        self.params['max_steer_angle'] = rospy.get_param('~max_steer_angle', 8.)
        self.params['min_speed'] = 0.0
        self.params['sample_rate'] = SAMPLE_RATE

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller = Controller(self.params)

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        self.dbw_enabled = False
        self.current_velocity = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.loop()

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def enabled_cb(self, msg):
        enabled = msg.data
        if enabled != self.dbw_enabled:
            rospy.loginfo("dbw_node: drive-by-wire changed to %s", enabled) 
        self.dbw_enabled = enabled

    def twist_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z

    def loop(self):
        rate = rospy.Rate(SAMPLE_RATE)
        while not rospy.is_shutdown():
            throttle, brake, steer = self.controller.control(
                linear_velocity=self.linear_velocity,
                angular_velocity=self.angular_velocity,
                current_velocity=self.current_velocity,
                enabled=self.dbw_enabled
            )
            rospy.loginfo("Requested velocity: %s" % (self.linear_velocity))
            rospy.loginfo("Current velocity: %s" % (self.current_velocity))
            rospy.loginfo("Thr: %s, Br: %s, St: %s" % (throttle, brake, steer))
            if self.dbw_enabled:
               self.publish(throttle, brake, steer)

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
