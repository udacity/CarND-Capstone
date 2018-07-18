#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Int32
import itertools
import numpy as np
from enum import Enum
from waypoint_updater import WaypointUpdater


class ControllerTuning(WaypointUpdater):

    def __init__(self):
        super(ControllerTuning, self).__init__()
        self.traffic_waypoint_msg = Int32(-1)  # Traffic waypoints are disabled
        self.set_next_tuning_pub = rospy.Publisher('/set_next_tuning', Empty, queue_size=1)
        self.tuning = TuningSettings(self.publish_set_next_tuning)
        self.waitUntilInit()
        self.loopForEver()

    def loopForEver(self):
        while not rospy.is_shutdown():
            if self.dbw_enabled_msg.data:
                # Get current tuning settings
                target_velocity, jerk_limit = self.tuning.get_settings(self.velocity_msg.twist.linear.x)

                # Update settings in waypoint calculator
                self.wp_calc.set_target_velocity(target_velocity)
                self.wp_calc.set_jerk_limit(jerk_limit)

                waypoints = self.wp_calc.calc_waypoints(self.pose_msg, self.velocity_msg, self.traffic_waypoint_msg)
                self.publish_waypoints(waypoints)
            else:
                assert(False)  # DBW must always be active when tuning
            self.rate.sleep()

    def publish_set_next_tuning(self):
        self.set_next_tuning_pub.publish(Empty())


class TuningSettings(object):

    def __init__(self, publish_set_next_tuning):
        self.publish_set_next_tuning = publish_set_next_tuning
        self.idx = 0
        self.state = self.State.ACCELERATE

        velocities = [20.0, 10.0, 5.0]
        jerk_limits = [10.0, 7.5, 5.0, 2.5]
        self.settings = list(itertools.product(velocities, jerk_limits))

    class State(Enum):
        ACCELERATE = 0
        KEEP_SPEED = 1
        DECELERATE = 2
        STAND_STILL = 3

    def get_settings(self, current_speed):
        if self.state == self.State.ACCELERATE and np.isclose(current_speed, self.settings[self.idx][0], atol=0.5):
            self.state = self.State.KEEP_SPEED
            self.start_time = rospy.get_time()
        elif self.state == self.State.KEEP_SPEED and (rospy.get_time() - self.start_time) > 5.0:
            self.state = self.State.DECELERATE
        elif self.state == self.State.DECELERATE and np.isclose(current_speed, 0.0, atol=0.005):
            self.state = self.State.STAND_STILL
            self.start_time = rospy.get_time()
        elif self.state == self.State.STAND_STILL and (rospy.get_time() - self.start_time) > 2.0:
            self.state = self.State.ACCELERATE
            self.idx += 1
            if self.idx == len(self.settings):
                self.publish_set_next_tuning()
                self.idx = 0

        if self.state in (self.State.ACCELERATE, self.State.KEEP_SPEED):
            target_speed = self.settings[self.idx][0]
        else:
            target_speed = 0.0

        jerk_limit = self.settings[self.idx][1]

        return target_speed, jerk_limit


if __name__ == '__main__':
    try:
        ControllerTuning()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
