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
from std_msgs.msg import Int32
from scipy.spatial import KDTree

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

# Number of waypoints we will publish.
_NUM_WAYPOINTS_AHEAD = 200
# Spin frequency in hertz.
_SPIN_FREQUENCY = 50
# Waypoint cushion from targeted stopline before traffic light or obstacle.
_STOP_CUSHION = 3
# Maximum deceleration.
_MAX_DECEL = .5


class WaypointUpdater(object):
    """
    This node publishes waypoints from the car's current position to some distance ahead.
    """

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_callback)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_callback)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_callback)

        # Publishers.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables.
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.traffic_light_wp_idx = None
        self.obstacle_wp_idx = None

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
        self.final_waypoints_pub.publish(self.get_final_lane(index))

    def get_final_lane(self, closest_wp_idx):
        """
        Updates final lane's waypoints based on traffic light or obstacle waypoint index.
        
        :return: lane with waypoints updated with decelerating linear velocity.
        """
        lane = Lane()
        lane.header = self.base_waypoints.header

        farthest_wp_idx = closest_wp_idx + _NUM_WAYPOINTS_AHEAD
        sliced_base_waypoints = self.base_waypoints.waypoints[closest_wp_idx:farthest_wp_idx]

        # Determine if vehicle is clear from traffic light and obstacle.
        traffic_light_clear = (self.traffic_light_wp_idx is None or
                               self.traffic_light_wp_idx == -1 or
                               self.traffic_light_wp_idx >= farthest_wp_idx)
        obstacle_clear = (self.obstacle_wp_idx is None or
                          self.obstacle_wp_idx == -1 or
                          self.obstacle_wp_idx >= farthest_wp_idx)

        if traffic_light_clear and obstacle_clear:
            # No traffic light or obstacle detected.
            lane.waypoints = sliced_base_waypoints
        else:
            if not traffic_light_clear and obstacle_clear:
                # Only traffic light is detected.
                target_wp_idx = self.traffic_light_wp_idx
            elif traffic_light_clear and not obstacle_clear:
                # Only obstacle is detected.
                target_wp_idx = self.obstacle_wp_idx
            else:
                # Both traffic light and obstacle are detected.
                target_wp_idx = min(self.traffic_light_wp_idx, self.obstacle_wp_idx)
            lane.waypoints = self.decelerate_waypoints(sliced_base_waypoints, target_wp_idx - closest_wp_idx)
        return lane

    @staticmethod
    def decelerate_waypoints(sliced_base_waypoints, stop_idx):
        """
        Loops through base waypoints to update the linear velocity base on deceleration with
        respect to the targeting stop waypoint.
        
        :return: list of waypoints with updated linear velocity.
        """
        decel_wp = []
        stop_idx = max(stop_idx - _STOP_CUSHION, 0)
        # Loop through each base_waypoint to adjust its linear velocity x.
        for i, wp in enumerate(sliced_base_waypoints):
            p = Waypoint()
            # Position of waypoint won't change.
            p.pose = wp.pose

            # To decelerate from speed v to 0 in a distance of s:
            # s = 1/2 * a * v^2  =>  v = sqrt(2 * a * s)
            dist = WaypointUpdater.waypoint_distance(sliced_base_waypoints, i, stop_idx)
            vel = math.sqrt(2 * _MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.

            WaypointUpdater.set_waypoint_velocity(p, min(vel, WaypointUpdater.get_waypoint_velocity(wp)))
            decel_wp.append(p)
        return decel_wp

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

    def traffic_callback(self, data):
        """
        Traffic waypoints subscriber callback function.
        """
        self.traffic_light_wp_idx = data

    def obstacle_callback(self, data):
        """
        Obstacle waypoints subscriber callback function.
        """
        self.obstacle_wp_idx = data

    @staticmethod
    def get_waypoint_velocity(waypoint):
        """
        Get the longitudinal velocity from a waypoint.
        """
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoint, velocity):
        """
        Sets the longitudinal velocity on a waypoint.
        """
        waypoint.twist.twist.linear.x = velocity

    @staticmethod
    def waypoint_distance(waypoints, wp1, wp2):
        """
        Gets piece-wise sum of the distances between adjacent waypoints.

        :param waypoints: waypoint list
        :param wp1: start index
        :param wp2: end index
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 - 1):
            dist += dl(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater().spin(_SPIN_FREQUENCY)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
