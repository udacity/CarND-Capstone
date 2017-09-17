#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter

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
        steer_ratio = 14.8
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        #init yaw_controller
        min_speed = 0.0
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # other variables:
        self.dbw_enabled = True

        # desired velocity
        self.des_linear_velocity = 0.0
        self.des_angular_velocity = 0.0

        # current velocity
        self.cur_linear_velocity = 0.0
        self.cur_angular_velocity = 0.0

        # init the low_pass filter
        tau = 1.5
        ts = 1.0
        self.lp_filter = LowPassFilter(tau, ts)

        # init the PID controller
        # TODO: init PID controller

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)


        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)  #geometry_msgs/TwistStamped
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)   #geometry_msgs/TwistStamped

        self.loop()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        return

    def current_velocity_cb(self, msg):
        self.cur_linear_velocity = msg.twist.linear.x
        self.cur_angular_velocity = msg.twist.angular.z

        #print("cur_linear_velocity = %s\n", self.cur_linear_velocity)
        return

    def dbw_enabled_cb(self, msg):
        # TODO: get the dbw_enabled
        return

    def twist_cmd_cb(self,msg):

        self.des_linear_velocity = msg.twist.linear.x
        self.des_angular_velocity = msg.twist.angular.z
        #print("des_linear_velocity = ", msg.twist.linear.x)
        #print("des_angular_velocity = ", msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)
        return

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():


            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            throttle = 0.50 # note throttle values should be in the range 0-1

            # TODO: use pid for acceleration

            # after getting the acceleration from PID, filter it using low_pass
            throttle = self.lp_filter.filt(throttle)

            # use yaw_controller for steering
            steer = self.yaw_control.get_steering(self.des_linear_velocity, self.des_angular_velocity, self.cur_linear_velocity)

            brake = 0

            #if <dbw is enabled>:

            if not self.dbw_enabled:
                rospy.loginfo("dbw is not enabled!")
                # TODO: RESET PID CONTROLLER

                pass
            else:
                self.publish(throttle, brake, steer)
                #print("accelerating with steer = ",steer)

            rate.sleep()
        return

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
