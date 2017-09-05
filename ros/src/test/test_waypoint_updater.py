#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
Test waypoint_updater via topics /final_waypoints
'''


class TestWaypointUpdater(object):
    def __init__(self):
        rospy.init_node('test_waypoint_updater')
        rospy.loginfo("Start waypoint updater test")

        # for debugging
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # for testing
        rospy.Subscriber('/final_waypoints', Lane, self.final_wp_cb)

        rospy.spin()

    def pose_cb(self, msg):
        rospy.loginfo("received position msg")

    def final_wp_cb(self, msg):
        rospy.loginfo("receiving msg ")



if __name__ == '__main__':
    try:
        TestWaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater tester node.')
