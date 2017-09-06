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
        # rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # rospy.Subscriber('/base_waypoints', Lane, self.base_wp_cb)

        # for testing
        rospy.Subscriber('/final_waypoints', Lane, self.final_wp_cb)

        rospy.spin()

    def pose_cb(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        rospy.loginfo("received position msg: x=%g, y=%g, z=%g" % (x, y, z))

    def base_wp_cb(self, msg):
        waypoints = msg.waypoints # 10902 waypoints
        # rospy.loginfo("%i waypoints" % len(waypoints))
        # x = waypoints[0].pose.pose.position.x
        # y = waypoints[0].pose.pose.position.y
        # z = waypoints[0].pose.pose.position.z
        # rospy.loginfo("first wp: x=%g, y=%g, z=%g" % (x, y, z))
        rospy.loginfo("base_waypoints header: %s" % str(msg.header))

    def final_wp_cb(self, msg):
        rospy.loginfo("Verifying /final_waypoints MSG")
        waypoints = msg.waypoints
        if len(waypoints) != 200:
            rospy.logdebug("number of waypoints should be = LOOKAHEAD_WPS (200)")



if __name__ == '__main__':
    try:
        TestWaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater tester node.')
