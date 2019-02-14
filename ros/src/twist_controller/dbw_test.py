#!/usr/bin/env python

import os
import csv

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport


'''
You can use this file to test your DBW code against a bag recorded with a reference implementation.
The bag can be found at https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/reference.bag.zip

To use the downloaded bag file, rename it to 'dbw_test.rosbag.bag' and place it in the CarND-Capstone/data folder.
Then with roscore running, you can then use roslaunch with the dbw_test.launch file found in 
<project_repo>/ros/src/twist_controller/launch.

This file will produce 3 csv files which you can process to figure out how your DBW node is
performing on various commands.

`/actual/*` are commands from the recorded bag while `/vehicle/*` are the output of your node.

'''


class DBWTestNode(object):
    def __init__(self):
        rospy.init_node('dbw_test_node', disable_signals=True)

        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steer_cb)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cb)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)

        rospy.Subscriber('/actual/steering_cmd', SteeringCmd, self.actual_steer_cb)
        rospy.Subscriber('/actual/throttle_cmd', ThrottleCmd, self.actual_throttle_cb)
        rospy.Subscriber('/actual/brake_cmd', BrakeCmd, self.actual_brake_cb)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.steer = self.throttle = self.brake = None

        self.steer_data = []
        self.throttle_data = []
        self.brake_data = []

        self.dbw_enabled = False

        base_path = os.path.dirname(os.path.abspath(__file__))
        self.steerfile = os.path.join(base_path, 'steers.csv')
        self.throttlefile = os.path.join(base_path, 'throttles.csv')
        self.brakefile = os.path.join(base_path, 'brakes.csv')

        # limits the max number of gathered samples
        # use None to not use the limit
        self.max_samples_steering = 1000
        self.max_samples_throttle = None
        self.max_samples_brake = None

        self.loop()

    def shutdown(self, reason):
        rospy.signal_shutdown(reason)

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        fieldnames = ['actual', 'proposed']

        with open(self.steerfile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.steer_data)

        with open(self.throttlefile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.throttle_data)

        with open(self.brakefile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.brake_data)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def steer_cb(self, msg):
        self.steer = msg.steering_wheel_angle_cmd

    def throttle_cb(self, msg):
        self.throttle = msg.pedal_cmd

    def brake_cb(self, msg):
        self.brake = msg.pedal_cmd

    def actual_steer_cb(self, msg):
        if self.max_samples_steering and len(self.steer_data) >= self.max_samples_steering:
            self.shutdown("Number of steering samples exceeds max_samples_steering=%d" % self.max_samples_steering)
            return

        if self.dbw_enabled and self.steer is not None:
            self.steer_data.append({'actual': msg.steering_wheel_angle_cmd,
                                    'proposed': self.steer})
            self.steer = None

    def actual_throttle_cb(self, msg):
        if self.max_samples_throttle and len(self.throttle_data) >= self.max_samples_throttle:
            self.shutdown("Number of throttle samples exceeds max_samples_throttle=%d" % self.max_samples_throttle)
            return

        if self.dbw_enabled and self.throttle is not None:
            self.throttle_data.append({'actual': msg.pedal_cmd,
                                       'proposed': self.throttle})
            self.throttle = None

    def actual_brake_cb(self, msg):
        if self.max_samples_brake and len(self.brake_data) >= self.max_samples_brake:
            self.shutdown("Number of brake samples exceeds max_samples_brake=%d" % self.max_samples_brake)
            return

        if self.dbw_enabled and self.brake is not None:
            self.brake_data.append({'actual': msg.pedal_cmd,
                                    'proposed': self.brake})
            self.brake = None


if __name__ == '__main__':
    DBWTestNode()
