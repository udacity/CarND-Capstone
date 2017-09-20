#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import math

from twist_controller import *

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
    """
    twist controller (DBW drive-by-wire)
    - subscribed topics:
        - `/current_velocity`
        - `/twist_cmd`
        - `/vehicle/dbw_enabled`
    - published topics:
        - `/vehicle/throttle_cmd`
        - `/vehicle/steering_cmd`
        - `/vehicle/brake_cmd`
    - node files:
        - `twist_controller/dbw_node.py`
    """
    def __init__(self):
        rospy.init_node('dbw_node')

        # parameters
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
        min_speed = 0

        # publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Controllers
        self.controller = Controller(
                                vehicle_mass,
                                wheel_radius,
                                accel_limit,
                                decel_limit,
                                wheel_base,
                                steer_ratio,
                                min_speed,
                                max_lat_accel,
                                max_steer_angle)

        # Subscribers
        # whether manual (dbw_enabled=false) or self driving
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)
        # twist command for car to control
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        # current car velocity from way points
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        

        # kept values
        self.dbw_enabled = None # whether mannual or self-driving
        self.car_twist = None # control car's twist based on its velocity
        self.car_velocity = None
        self.last_timestamp = rospy.rostime.get_time()
        # self.last_throttle = None
        # self.flag = None

        self.loop()

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data

    def twist_cb(self, msg):
        self.car_twist = msg.twist

    def velocity_cb(self, msg):
        self.car_velocity = msg.twist

    def loop(self):
        rate = rospy.Rate(50) # 50Hz is too high for testing
        while not rospy.is_shutdown():
            # deal with time
            now = rospy.rostime.get_time()
            dt = now - self.last_timestamp
            self.last_timestamp = now

            # if got signal to control
            if self.car_twist is not None and self.car_velocity is not None:
                target_velocity = (self.car_twist.linear.x, 
                                    self.car_twist.angular.z)
                current_velocity = (self.car_velocity.linear.x,
                                    self.car_velocity.angular.z)

                throttle, brake, steering = self.controller.control(
                    target_velocity,
                    current_velocity,
                    self.dbw_enabled,
                    dt
                )

            # you should only publish control if dbw is enabled
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):

        if brake <= 0:        
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

            scmd = SteeringCmd()
            scmd.enable = True
            scmd.steering_wheel_angle_cmd = steer
            self.steer_pub.publish(scmd)

        else:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)    


if __name__ == '__main__':
    DBWNode()
