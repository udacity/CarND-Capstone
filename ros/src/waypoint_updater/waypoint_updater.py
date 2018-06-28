#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
from scipy.spatial import KDTree
import numpy as np
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
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.max_velocity = None
        self.base_waypoints_msg = None
        self.wp_search = None
        self.rate = rospy.Rate(50)
        self.waitUntilInit()
        self.loopForEver()

    def waitUntilInit(self):
        """Wait until all subscriptions has provided at least one msg."""
        while not rospy.is_shutdown():
            if None not in (self.pose, self.base_waypoints_msg):
                # All base waypoints should have the same velocity (in this project).
                self.max_velocity = self.get_waypoint_velocity(self.base_waypoints_msg.waypoints[0])
                # Just make sure the assumption above is correct.
                assert(all(self.max_velocity == self.get_waypoint_velocity(waypoint)
                           for waypoint in self.base_waypoints_msg.waypoints))
                self.wp_search = WaypointSearch(self.base_waypoints_msg.waypoints)
                break
            self.rate.sleep()

    def loopForEver(self):
        while not rospy.is_shutdown():
            # Get closest waypoint
            closest_waypoint_idx = self.wp_search.get_closest_waypoint_ahead_idx(self.pose)
            self.publish_waypoints(closest_waypoint_idx)
            self.rate.sleep()

    def publish_waypoints(self, closest_idx):
        final_waypoints = Lane()
        final_waypoints.header = self.base_waypoints_msg.header
        final_waypoints.waypoints = self.base_waypoints_msg.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.keep_speed_limit(final_waypoints)
        self.final_waypoints_pub.publish(final_waypoints)

    def keep_speed_limit(self, final_waypoints):
        """ Makes sure we never exceeds max velocity """
        for i in range(len(final_waypoints.waypoints)):
            velocity = self.get_waypoint_velocity(final_waypoints.waypoints[i])
            if velocity > self.max_velocity:
                # Note that this is not expected to ever happen. However it's perhaps not a fatal error since the
                # velocity can be limited to the max allowed value here.
                rospy.logerr('Calculated velocity %s exceeds max allowed value %s.', velocity, self.max_velocity)
                self.set_waypoint_velocity(final_waypoints.waypoints, i, self.max_velocity)

    def pose_cb(self, msg):
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints_msg = waypoints

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
            return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


class WaypointSearch(object):
    """Organize waypoints to make it fast to search for the closest to a position"""
    def __init__(self, waypoints):
        # Preprocess waypoints using the k-d tree algorithm
        self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_ahead_idx(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
