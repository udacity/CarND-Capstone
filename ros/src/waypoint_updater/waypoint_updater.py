#!/usr/bin/env python
"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
"""
import math

import numpy as np
import rospy
from scipy.spatial import KDTree

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

# Number of waypoints we will publish.
_NUM_WAYPOINTS_AHEAD = 200
# Spin frequency in hertz.
_SPIN_FREQUENCY = 50


class WaypointUpdater(object):
    """
    This node publishes waypoints from the car's current position to some `x` distance ahead.
    """

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_callback)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Publishers.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables.
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None

    def spin(self, freq):
        """
        Spins this ROS node based on the given frequency.

        :param freq: frequency in hertz.
        """
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get the closest waypoint and publish it.
                self.publish_waypoints(self.get_closest_waypoint_idx())
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        Gets the index of the closest waypoint.

        :return: index of the closest waypoint.
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # The first 1 is for closest. The second 1 is for the index element.
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if the closest waypoint is ahead or behind the ego car.
        closest_2d = self.waypoints_2d[closest_idx]
        prev_2d = self.waypoints_2d[closest_idx - 1]
        closest_vect = np.array(closest_2d)
        prev_vector = np.array(prev_2d)
        curr_vector = np.array([x, y])
        if np.dot(closest_vect - prev_vector, curr_vector - closest_vect) > 0:
            # The closest waypoint is behind. Pick the next index.
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, index):
        """
        Publishes the waypoints to ROS.

        :param index of the first waypoint.
        """
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[index:index + _NUM_WAYPOINTS_AHEAD]
        self.final_waypoints_pub.publish(lane)

    def pose_callback(self, pose):
        """
        Pose subscriber callback function.
        """
        self.pose = pose

    def base_waypoints_callback(self, base_waypoints):
        """
        Base waypoints subscriber callback function.

        The publisher has latch set to True, which means this message will be received only once.
        """
        # Get the waypoints in X, Y plane and set up the KDTree for efficient comparison.
        self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y]
                             for w in base_waypoints.waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)
        self.base_waypoints = base_waypoints
        rospy.loginfo('base_waypoints initialized')

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
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater().spin(_SPIN_FREQUENCY)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
