#!/usr/bin/env python
import sys
import os
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.kdtree import KDTree
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.pose = None
        self.base_waypoint = None
        self.waypoints_tree = None
        self.spin_rate = 50

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.spin()

    def spin(self):
        rate = rospy.Rate(self.spin_rate)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoint:
                next_way_point_idx = self.get_next_waypoint()
                self.publish_waypoint(next_way_point_idx)
            rate.sleep()

    def get_next_waypoint(self):
        pose_x = self.pose.pose.position.x
        pose_y = self.pose.pose.position.y

        closest_ids = self.waypoints_tree.query([pose_x, pose_y], 1)[1]

        closest_coords = self.waypoints_tree.data[closest_ids]
        prev_coords = self.waypoints_tree.data[closest_ids - 1]

        cl_vect = np.array(closest_coords)
        prev_vect = np.array(prev_coords)
        pose_vect = np.array([pose_x, pose_y])

        val = np.dot(cl_vect - prev_vect, pose_vect - cl_vect)
        if val > 0:
            closest_ids = (closest_ids + 1) % len(self.waypoints_tree.data)
        return closest_ids



    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoint.header
        lane.waypoints = self.base_waypoint.waypoints[closest_idx: closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoint = waypoints
        if not self.waypoints_tree:
            waypoint_coord = [[waypoint.pose.pose.position.x, waypoints.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(waypoint_coord)

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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    import sys
    print sys.path
    # try:
    #     WaypointUpdater()
    # except rospy.ROSInterruptException:
    #     rospy.logerr('Could not start waypoint updater node.')
