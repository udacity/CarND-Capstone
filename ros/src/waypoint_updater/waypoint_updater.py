#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables to store values from callback functions
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1

        self.loop()

    def loop(self):
        # control publishing frequency
        # waypoint follower has frequency around 30 Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                self.publish_waypoints()

            rate.sleep()

    def get_closest_waypoint_idx(self):
        # load current position and find closest waypoint by KDTree
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest i ahead or behind vehilcle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        # extract waypoints starting from closest index with the number of
        # points we want to look ahead.
        closest_wp_idx = self.get_closest_waypoint_idx()
        farthest_wp_idx = closest_wp_idx + LOOKAHEAD_WPS
        waypoints = self.base_waypoints.waypoints[closest_wp_idx:farthest_wp_idx]

        lane = Lane()
        lane.header = self.base_waypoints.header

        # if the next stop line is not in the waypoint range, we ignore it
        if self.stopline_wp_idx == -1 or \
           (self.stopline_wp_idx >= farthest_wp_idx):
            lane.waypoints = waypoints
        # otherwise, we want to decelerate and stop at the point
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints, closest_wp_idx)

        self.final_waypoints_pub.publish(lane)

    def decelerate_waypoints(self, waypoints, closest_idx):

        # two waypoints back from the stop line, so front of car stops at the line
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
        # create new waypoint points that stops at the stop line
        new_wps = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_idx)
            # decrease velocity based on distance to the stop line
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # stop the car when the velocity is small TODO: modify this
            vel = vel if vel >= 1. else 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            new_wps.append(p)
        return new_wps

    def pose_cb(self, msg):
        # about 50 Hz
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # load base waypoints
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            # convert waypoints to (x,y) list
            self.waypoints_2d = [
                [
                    waypoint.pose.pose.position.x,
                    waypoint.pose.pose.position.y
                ] for waypoint in waypoints.waypoints
            ]
            # build KDTree
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

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
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
