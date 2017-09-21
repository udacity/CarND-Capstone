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

        params = {}
        params['vehicle_mass'] = rospy.get_param('~vehicle_mass', 1736.35)
        params['fuel_capacity'] = rospy.get_param('~fuel_capacity', 13.5)
        params['brake_deadband'] = rospy.get_param('~brake_deadband', .1)
        params['decel_limit'] = rospy.get_param('~decel_limit', -5)
        params['accel_limit'] = rospy.get_param('~accel_limit', 1.)
        params['wheel_radius'] = rospy.get_param('~wheel_radius', 0.2413)
        params['wheel_base'] = rospy.get_param('~wheel_base', 2.8498)
        params['steer_ratio'] = rospy.get_param('~steer_ratio', 14.8)
        params['max_lat_accel'] = rospy.get_param('~max_lat_accel', 3.)
        params['max_steer_angle'] = rospy.get_param('~max_steer_angle', 8.)

        self.curr_linear_velocity =0
        self.curr_angular_velocity = 0
        self.dbw_enabled = False
        self.target_linear_velocity = 0
        self.target_angular_velocity = 0
        
        self.rate = 10

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(**params)

        # TODO: Subscribe to all the topics you need to
        self.curr_vel_subscriber  = rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_Cb)
        self.dbw_enable_subscriber  = rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_Cb)
        self.twist_cmd_subscriber  = rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_Cb)
        self.loop()

    

    def loop(self):
        rate = rospy.Rate(self.rate) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
              
             throttle, brake, steer = self.controller.control(self.target_linear_velocity,
                     self.target_angular_velocity,
                     self.curr_linear_velocity,
                     self.curr_angular_velocity,
                     self.dbw_enabled)

             rospy.loginfo("--- throttle=%s, brake=%s, steer=%s", throttle,brake,steer)
             if self.dbw_enabled is True:
                 rospy.logwarn("---dbw_enabled=%s", self.dbw_enabled)
                 self.publish(throttle, brake, steer)
             rate.sleep()

    def current_velocity_Cb(self, data):
        self.curr_linear_velocity = data.twist.linear.x
        self.curr_angular_velocity = data.twist.angular.z

    def dbw_enabled_Cb(self,data):
        self.dbw_enabled = data.data

    def twist_cmd_Cb(self,data):
        self.target_linear_velocity = data.twist.linear.x # 4.47 for 10mph
        self.target_angular_velocity = data.twist.angular.z

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
