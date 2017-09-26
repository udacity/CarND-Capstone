#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint #, Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        self.waypoints = None

        rospy.logdebug("WaypointUpdater started")
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        rospy.logdebug("pose_cb fired: %s", msg.header.stamp)
        if self.waypoints is not None: self.find_closest_publish_lane(msg)
        pass

    def waypoints_cb(self, waypoints):
        rospy.logdebug("waypoints_cb fired")
        self.waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        rospy.logdebug("traffic_cb fired")
        pass

    def obstacle_cb(self, msg):
        rospy.logdebug("obstacle_cb fired")
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def find_closest_publish_lane(self, pose):
        waypoints = self.waypoints.waypoints

        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        i_min = -1
        dist_min = float('inf')
        for i, wp in enumerate(waypoints):
            dist = dl(pose.pose.position, wp.pose.pose.position)
            if dist < dist_min:
                dist_min = dist
                i_min = i

        if i_min < 0:
            rospy.logwarn('No waypoints ahead')
            return

        rest_wp = waypoints[i_min: min(i_min + LOOKAHEAD_WPS, len(waypoints))]
        while len(rest_wp) < LOOKAHEAD_WPS:
            rest_wp += waypoints[:min(LOOKAHEAD_WPS-len(rest_wp), len(waypoints))]

        lane = Lane()
        lane.header = self.waypoints.header
        lane.waypoints = rest_wp
        self.final_waypoints_pub.publish(lane)
        rospy.loginfo('lane publish from %s index', i_min)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
