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
DBW_UPDATE_RATE = 15

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # log info for debug
        rospy.loginfo("vehicle_mass = %s",self.vehicle_mass)
        rospy.loginfo("fuel_capacity = %s", self.fuel_capacity)
        rospy.loginfo("brake_deadband = %s", self.brake_deadband)
        rospy.loginfo("decel_limit = %s", self.decel_limit)
        rospy.loginfo("accel_limit = %s", self.accel_limit)
        rospy.loginfo("wheel_radius = %s", self.wheel_radius)
        rospy.loginfo("wheel_base = %s", wheel_base)
        rospy.loginfo("steer_ratio = %s", steer_ratio)
        rospy.loginfo("max_lat_accel = %s", max_lat_accel)
        rospy.loginfo("max_steer_angle = %s", max_steer_angle)

        #init yaw_controller
        min_speed = 0.0
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # drive by wire enables set to False in the beginning
        self.dbw_enabled = False

        # desired velocity
        self.des_linear_velocity = 0.0
        self.des_angular_velocity = 0.0

        # current velocity
        self.cur_linear_velocity = 0.0
        self.cur_angular_velocity = 0.0

        # watchdogs for safety critical handling
        self.current_timestamp = rospy.Time.now()
        self.vel_timestamp = rospy.Time.now()
        self.twist_cmd_timestamp = rospy.Time.now()
        # TODO verify if this value is OK
        self.watchdog_limit = 0.5e9 # half second

        # init the low_pass filter
        tau = 1.5
        ts = 1.0
        self.lp_filter = LowPassFilter(tau, ts)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

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
        self.vel_timestamp = rospy.Time.now()
        return

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo("Update self.dbw_enabled=%s", self.dbw_enabled)
        return

    def twist_cmd_cb(self,msg):
        # TODO: verify if this hack doesn't cause any other problems
        # discard negative linear speeds
        if msg.twist.linear.x < 0:
            return

        self.des_linear_velocity = msg.twist.linear.x
        self.des_angular_velocity = msg.twist.angular.z
        self.twist_cmd_timestamp = rospy.Time.now()
        return

    def loop(self):

        rate = rospy.Rate(DBW_UPDATE_RATE) # 50Hz
        rospy.loginfo("DBW running with update rate = %s",DBW_UPDATE_RATE)

        # initialise a controller
        self.throttle_control = Controller(pid_kp=0.8, pid_ki=0.25, pid_kd=0.1,
                                           min_value=self.decel_limit, max_value=self.accel_limit)

        while not rospy.is_shutdown():

            # handle safety critical failures
            self.current_timestamp = rospy.Time.now()

            #rospy.loginfo("vel_gap = %s, cmd_gap = %s",(self.current_timestamp - self.vel_timestamp).nsecs,(self.current_timestamp - self.twist_cmd_timestamp).nsecs)
            if (self.current_timestamp - self.vel_timestamp).nsecs > self.watchdog_limit:
                # stop the car
                rospy.logwarn("Safety hazard: not receiving VEL info for long time. Stopping the vehicle!")
                self.des_linear_velocity = 0
                
            if  (self.current_timestamp - self.twist_cmd_timestamp).nsecs > self.watchdog_limit:
                # stop the car
                rospy.logwarn("Safety hazard: not receiving TWIST_CMD info for long time. Stopping the vehicle!")
                self.des_linear_velocity = 0

            # pid for acceleration
            throttle, brake, steering = self.throttle_control.control(self.des_linear_velocity,
                                                                      self.cur_linear_velocity, self.dbw_enabled)


            if throttle < 0.001: # very small number
                # TODO: maybe verify the units
                if throttle < -self.brake_deadband:
                    brake = self.vehicle_mass * abs(throttle) * self.wheel_radius
                throttle = 0
            else:
                brake = 0

            # after getting the acceleration from PID, filter it using low_pass
            throttle = self.lp_filter.filt(throttle)

            # use yaw_controller for steering
            # TODO: try using a low pass filter for angular velocity to smooth the steering
            steer = self.yaw_control.get_steering(self.des_linear_velocity, self.des_angular_velocity, self.cur_linear_velocity)

            if self.dbw_enabled:
                self.publish(throttle, brake, steer)
                #rospy.loginfo("ref_linear_vel = %s \tcur_linear_vel = %s \tthrottle = %s \tbrake = %s \t",
                #              self.des_linear_velocity, self.cur_linear_velocity, throttle, brake)

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
