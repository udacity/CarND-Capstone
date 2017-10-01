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

class VehicleConfiguration(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        self.min_speed = None

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vc = VehicleConfiguration()

        vc.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        vc.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        vc.brake_deadband = rospy.get_param('~brake_deadband', .1)
        vc.decel_limit = rospy.get_param('~decel_limit', -1.)
        vc.accel_limit = rospy.get_param('~accel_limit', 1.)
        vc.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        vc.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        vc.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        vc.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        vc.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        vc.min_speed = 0.0 # TODO param

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        #self.controller = Controller(kp=0.4, ki=0.0, kd=0.5, vc=vc) # working: car stays on track, but throttle oscillates a lot
        # self.controller = Controller(kp=0.2, ki=0.001, kd=0.15, vc=vc) # sort of OK
        # self.controller = Controller(kp=0.1, ki=0.0, kd=0.05, vc=vc) # fits expectation better
        # self.controller = Controller(kp=0.1, ki=0.0, kd=0.03, vc=vc) # fits expectation even better
        # self.controller = Controller(kp=0.1, ki=0.0, kd=0.02, vc=vc) # fits expectation even better - car overshoots
        # self.controller = Controller(kp=0.1, ki=0.0, kd=0.025, vc=vc) # good
        # self.controller = Controller(kp=0.15, ki=0.001, kd=0.025, vc=vc) #
        # self.controller = Controller(kp=0.9, ki=0.0, kd=0.0, vc=vc) # good
        # self.controller = Controller(kp=1.0, ki=0.0, kd=0.0, vc=vc) #
        # self.controller = Controller(kp=1.6, ki=0.0, kd=0.01, vc=vc) # good
        # self.controller = Controller(kp=1.6, ki=0.0, kd=0.04, vc=vc) # good ++++++++++++++
        # self.controller = Controller(kp=1.6, ki=0.00005, kd=0.04, vc=vc) # good ++++++++++++
        self.controller = Controller(kp=1.6, ki=0.00001, kd=0.04, vc=vc) # good



        self.dbw_enabled = False
        self.controller_reset = False
        self.current_velocity = None
        self.latest_twist_cmd = None

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.set_dbw_enabled)
        rospy.Subscriber('/current_velocity', TwistStamped, self.set_current_velocity)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.set_twist_cmd)

        self.loop()

    def set_dbw_enabled(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled

    def set_current_velocity(self, current_velocity):
        self.current_velocity = current_velocity

    def set_twist_cmd(self, twist_cmd):
        self.latest_twist_cmd = twist_cmd

    def loop(self):
        rate = rospy.Rate(10) # 50Hz ###################################################################

        while not rospy.is_shutdown():

            rospy.loginfo("""DBW enabled: {}""".format(self.dbw_enabled))

            if self.dbw_enabled and self.current_velocity is not None and self.latest_twist_cmd is not None:
                if not self.controller_reset:
                    self.controller.reset()
                    self.controller_reset = True


                # TODO re-add this output
                #rospy.loginfo("""Velocity Ref: {} - Curr: {} - Err: {}""".format(self.ref_velocity, self.current_velocity, velocity_error))

                throttle, brake, steer = self.controller.control(self.current_velocity, self.latest_twist_cmd)

                rospy.loginfo("""Throttle Brake Steer: {} {} {}""".format(throttle, brake, steer))

                self.publish(throttle, brake, steer)
            else:
                self.controller_reset = False
            rate.sleep()

    def publish(self, throttle, brake, steer):

        #if (throttle > 0.0):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        #if (brake > 0.0):
        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_PERCENT
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
