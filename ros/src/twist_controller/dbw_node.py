#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from controller import Controller

from dynamic_reconfigure.server import Server
from twist_controller.cfg import PidGainsConfig

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

        # fetch parameters
        self.ros_params = {
            'vehicle_mass' : rospy.get_param('~vehicle_mass', 1736.35),
            'fuel_capacity' : rospy.get_param('~fuel_capacity', 13.5),
            'brake_deadband' : rospy.get_param('~brake_deadband', .1),
            'decel_limit' : rospy.get_param('~decel_limit', -5),
            'accel_limit' : rospy.get_param('~accel_limit', 1.),
            'wheel_radius' : rospy.get_param('~wheel_radius', 0.2413),
            'wheel_base' : rospy.get_param('~wheel_base', 2.8498),
            'steer_ratio' : rospy.get_param('~steer_ratio', 14.8),
            'max_lat_accel' : rospy.get_param('~max_lat_accel', 3.),
            'max_steer_angle' : rospy.get_param('~max_steer_angle', 8.)
        }

        # publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # create `Controller` object
        self.controller = Controller(**self.ros_params)

        # subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.cb_dbw_enabled, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cb_current_velocity, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.cb_twist_cmd, queue_size=1)

        # activate dynamic configuration of the module
        drc_srv = Server(PidGainsConfig, self.cb_dynamic_config)

        # variables
        self.dbw_enabled = False
        self.cur_vel_linear = 0
        self.cur_vel_angular = 0
        self.req_vel_linear = 0
        self.req_vel_angular = 0

        # main loop
        self.loop()

    def cb_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo('cb_dbw_enabled: self.dbw_enabled = {}'.format(self.dbw_enabled))

    def cb_current_velocity(self, msg):
        self.cur_vel_linear = msg.twist.linear.x
        self.cur_vel_angular = msg.twist.angular.z

    def cb_twist_cmd(self, msg):
        self.req_vel_linear = msg.twist.linear.x
        self.req_vel_angular = msg.twist.angular.z

    def cb_dynamic_config(self, config, level):
        rospy.loginfo("New gains for throttle-PID: {throttle_Kp}, {throttle_Ki}, {throttle_Kd}".format(**config))
        self.controller.update_throttle_gains( config['throttle_Kp'], config['throttle_Ki'], config['throttle_Kd'])
        return config

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # get predicted throttle, brake and steering from the twist_controller
            throttle, brake, steering = self.controller.control(rospy.get_time(),
                                                                self.req_vel_linear, 
                                                                self.req_vel_angular,
                                                                self.cur_vel_linear,
                                                                self.cur_vel_angular,
                                                                self.dbw_enabled)

            # only publish if dbw is enabled
            if self.dbw_enabled:
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
