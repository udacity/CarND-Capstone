#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy import spatial
from typing import List

import math

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater:

    _base_waypoints = None
    _waypoints_2d = None
    _waypoints_tree = None

    _pose = None
    _frequency = 50  # Hertz

    def __init__(self):
        rospy.init_node("waypoint_updater")

        rospy.Subscriber("/current_pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self.waypoints_cb)
        rospy.Subscriber("/traffic_waypoint", Lane, self.traffic_cb)
        rospy.Subscriber("/obstacle_waypoint", Lane, self.obstacle_cb)

        # `final_waypoints` are published to way point follower (part of Autoware).
        self._final_waypoints_pub = rospy.Publisher(
            "final_waypoints", Lane, queue_size=1
        )

        # TODO: Add other member variables you need below
        self.start()

    def start(self):
        rate = rospy.Rate(self._frequency)
        while not rospy.is_shutdown():
            if self._pose is not None and self._base_waypoints is not None:
                closest_waypoint_index = self._get_closest_waypoint_index(
                    point=[self._pose.pose.position.x, self._pose.pose.position.y]
                )
                self._publish_waypoints(closest_waypoint_index=closest_waypoint_index)
            rate.sleep()

    def _publish_waypoints(self, *, closest_waypoint_index: int):
        lane = Lane()
        lane.header = self._base_waypoints.header
        lane.waypoints = self._base_waypoints.waypoints[
            closest_waypoint_index : closest_waypoint_index + LOOKAHEAD_WPS
        ]
        self._final_waypoints_pub.publish(lane)

    def _get_closest_waypoint_index(self, *, point: np.ndarray | List[float]) -> int:
        closest_index = -1
        if self._waypoints_tree is not None:
            _, closest_index = self._waypoints_tree.query(point, k=1)
            closest_point = self._waypoints_2d[closest_index]
            prev_point = self._waypoints_2d[closest_index - 1]

            direction = np.dot(closest_point - prev_point, point - closest_point)
            if direction > 0:
                closest_index = (closest_index + 1) % len(self._waypoints_2d)
        return closest_index

    def pose_cb(self, msg):
        # TODO: Implement
        self._pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self._base_waypoints = waypoints
        if self._waypoints_2d is not None:
            self._waypoints_2d = np.asarray(
                [
                    [p.pose.pose.position.x, p.pose.pose.position.y]
                    for p in self._base_waypoints.waypoints
                ]
            )
            self._waypoints_tree = spatial.KDTree(self._waypoints_2d)
        # TODO: Implement
        pass

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
        dl = lambda a, b: math.sqrt(
            (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
        )
        for i in range(wp1, wp2 + 1):
            dist += dl(
                waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position
            )
            wp1 = i
        return dist


if __name__ == "__main__":
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start waypoint updater node.")
