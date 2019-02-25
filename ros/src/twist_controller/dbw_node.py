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
class CarParameters(object):
    def __init__(self):

        vehicle_mass = None
        fuel_capacity = None
        brake_deadband = None
        decel_limit = None
        accel_limit = None
        wheel_radius = None
        wheel_base = None
        steer_ratio = None
        max_lat_accel = None
        max_steer_angle = None


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        car_prm = CarParameters()

        car_prm.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        car_prm.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        car_prm.brake_deadband = rospy.get_param('~brake_deadband', .1)
        car_prm.decel_limit = rospy.get_param('~decel_limit', -5)
        car_prm.accel_limit = rospy.get_param('~accel_limit', 1.)
        car_prm.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        car_prm.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        car_prm.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        car_prm.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        car_prm.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        
        self.twist_cmd_new = None
        self.curr_vel = None
        self.dbw_enbl = None
        self.throttle = self.brake = self.steering = 0
        # TODO: Create `Controller` object
        self.controller = Controller(car_prm)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb, queue_size = 5)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size = 5)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enbl_cb, queue_size = 1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            self.throttle, self.brake, self.steering = self.controller.control(self.curr_vel, self.twist_cmd_new,
                                                                self.dbw_enbl)
            if self.dbw_enbl:
               self.publish(self.throttle, self.brake, self.steering)
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


    def twist_cmd_cb(self, msg):
        self.twist_cmd_new = msg

    def curr_vel_cb(self, msg):
        self.curr_vel = msg

    def dbw_enbl_cb(self, msg):
        self.dbw_enbl = msg

if __name__ == '__main__':
    DBWNode()
