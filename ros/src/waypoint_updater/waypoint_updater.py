#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber(
            '/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)
        self.base_waypoints = []
        # TODO: Add other member variables you need below

        rospy.spin()

    def straight_dist(self, pos0, pos1):
        return math.sqrt((pos0.x - pos1.x) ** 2 + (pos0.y - pos1.y) ** 2)

    def closest_waypoint(self, curr_pose):
        closest_distance = float('inf')
        closest_idx = -1
        for idx, waypoint in enumerate(self.base_waypoints):
            distance = self.straight_dist(
                waypoint.pose.pose.position,
                curr_pose.pose.position)
            if distance < closest_distance:
                closest_idx = idx
                closest_distance = distance

        return closest_idx

    def pose_cb(self, pose_stamped):
        current_pose_idx = self.closest_waypoint(pose_stamped)
        final_waypoints_msg = Lane()
        final_waypoints_msg.waypoints = self.base_waypoints[
            current_pose_idx:current_pose_idx+LOOKAHEAD_WPS
        ]
        self.final_waypoints_pub.publish(final_waypoints_msg)

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b):
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
