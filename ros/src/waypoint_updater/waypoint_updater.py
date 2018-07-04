#!/usr/bin/env python

import rospy
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math
import yaml
from speed_calculator import SpeedCalculator

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
STOPLINE_WPS_MARGIN = 2  # Number of waypoints to use as a safety margin when stopping at traffic lights.


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose_msg = None
        self.velocity_msg = None
        self.max_velocity = None
        self.base_waypoints_msg = None
        self.traffic_lights_msg = None
        self.wp_search = None
        self.stop_line_wp_indices = None
        self.rate = rospy.Rate(50)

        self.waitUntilInit()
        self.loopForEver()

    def waitUntilInit(self):
        """Wait until all subscriptions has provided at least one msg."""
        while not rospy.is_shutdown():
            if None not in (self.pose_msg, self.velocity_msg, self.base_waypoints_msg, self.traffic_lights_msg):
                # All base waypoints should have the same velocity (in this project).
                self.max_velocity = self.get_waypoint_velocity(self.base_waypoints_msg.waypoints[0])
                # Just make sure the assumption above is correct.
                assert(all(self.max_velocity == self.get_waypoint_velocity(waypoint)
                           for waypoint in self.base_waypoints_msg.waypoints))
                self.wp_search = WaypointSearch(self.base_waypoints_msg.waypoints)
                self.stop_line_wp_indices = [self.wp_search.get_closest_waypoint_idx_behind(x, y) - STOPLINE_WPS_MARGIN
                                             for x, y in self.load_stop_line_positions()]
                break
            self.rate.sleep()

    def load_stop_line_positions(self):
        traffic_light_config_string = rospy.get_param("/traffic_light_config")
        traffic_light_config = yaml.load(traffic_light_config_string)
        return traffic_light_config['stop_line_positions']

    def loopForEver(self):
        previous_first_idx = None
        waypoints = None
        while not rospy.is_shutdown():
            # Extract base_waypoints ahead of vehicle as a starting point for the final_waypoints.
            first_idx = self.wp_search.get_closest_waypoint_idx_ahead(self.pose_msg.pose.position.x,
                                                                      self.pose_msg.pose.position.y)
            if previous_first_idx is None:
                # Construct previous waypoint from the vehicles initial pose and velocity.
                preceding_waypoint = Waypoint()
                preceding_waypoint.pose.pose = self.pose_msg.pose
                preceding_waypoint.twist.twist = self.velocity_msg.twist
            elif previous_first_idx != first_idx:
                # Copy the preceding waypoint before updating the waypoint list.
                preceding_waypoint = waypoints[first_idx - previous_first_idx - 1]

            waypoints = deepcopy(self.base_waypoints_msg.waypoints[first_idx:first_idx + LOOKAHEAD_WPS])
            previous_first_idx = first_idx

            # Calculate the final_waypoints
            self.accelerate_to_speed_limit(preceding_waypoint, waypoints)
            self.adjust_for_traffic_lights(waypoints, first_idx)
            self.assert_speed_limit(waypoints)

            self.publish_waypoints(waypoints)
            self.rate.sleep()

    def accelerate_to_speed_limit(self, preceding_waypoint, waypoints):
        """Set waypoints speed to accelerate the vehicle to max velocity.

        Parameters
        ----------
        preceding_waypoint : The last waypoint already visited, e.g. the nearest one behind the vehicle.
        waypoints : The waypoints to follow ahead of the vehicle.

        """

        # Base the acceleration on the velocity from preceding waypoint.
        current_speed = self.get_waypoint_velocity(preceding_waypoint)
        speed_calc = SpeedCalculator(target_speed=self.max_velocity, current_speed=current_speed,
                                     target_acceleration=0.0, current_accleration=0.0,
                                     acceleration_limit=10.0, jerk_limit=10.0)
        distances = self.calc_distances(preceding_waypoint, waypoints)
        for idx in range(len(waypoints)):
            speed = speed_calc.get_speed_at_distance(distances[idx])
            self.set_waypoint_velocity(waypoints, idx, speed)

    def adjust_for_traffic_lights(self, waypoints, first_idx):
        """Adjust the speed to break for traffic lights ahead that not is at state green.

        Parameters
        ----------
        waypoints : The waypoints to be adjusted.
        first_idx : The base_waypoint index from where waypoints have been derived.
        """

        # Check if there is a stop-line along the range of waypoints
        for base_idx in range(first_idx - STOPLINE_WPS_MARGIN, first_idx + LOOKAHEAD_WPS):
            if base_idx in self.stop_line_wp_indices:
                tl_idx = self.stop_line_wp_indices.index(base_idx)
                if self.traffic_lights_msg.lights[tl_idx].state is not TrafficLight.GREEN:
                    final_idx = max(0, base_idx - first_idx)
                    rospy.loginfo("Stopping at base_idx=%s final_idx=%s", base_idx, final_idx)
                    self.stop_at_waypoint(final_idx, waypoints)
                    break
        else:
            rospy.loginfo("No speed adjustments for traffic lights.")

    def stop_at_waypoint(self, stop_idx, waypoints):
        """ Reduce speed to stop at stop_idx.

        Parameters
        ----------
        stop_idx : The waypoint index where the vehicle should reach a speed of zero.
        waypoints : The waypoints to be adjusted.
        """

        # Set the velocity for all waypoints after the stop_idx to zero.
        for idx in range(stop_idx, len(waypoints)):
            self.set_waypoint_velocity(waypoints, idx, 0.0)

        # Calculate the deceleration backwards from the stop_idx,
        # until reaching a speed that is greater than what was already requested.
        speed = 0.0
        acceleration = -1.0  # Speed change per waypoint
        for idx in range(stop_idx, -1, -1):
            self.set_waypoint_velocity(waypoints, idx, speed)
            acceleration = min(-0.1, acceleration * 0.9)
            speed -= acceleration
            if speed > self.get_waypoint_velocity(waypoints[idx - 1]):
                break

    def assert_speed_limit(self, waypoints):
        """Makes sure we never exceeds max velocity"""
        for i in range(len(waypoints)):
            velocity = self.get_waypoint_velocity(waypoints[i])
            if velocity > self.max_velocity:
                # Note that this is not expected to ever happen. However it's perhaps not a fatal error since the
                # velocity can be limited to the max allowed value here.
                rospy.logerr('Calculated velocity %s exceeds max allowed value %s.', velocity, self.max_velocity)
                self.set_waypoint_velocity(waypoints, i, self.max_velocity)

    def publish_waypoints(self, waypoints):
        final_waypoints_msg = Lane()
        final_waypoints_msg.header = self.base_waypoints_msg.header
        final_waypoints_msg.waypoints = waypoints
        self.final_waypoints_pub.publish(final_waypoints_msg)

    def pose_cb(self, msg):
        """Callback for /current_pose topic"""
        self.pose_msg = msg

    def velocity_cb(self, msg):
        """Callback for /current_velocity topic"""
        self.velocity_msg = msg

    def waypoints_cb(self, msg):
        """Callback for /base_waypoints topic"""
        self.base_waypoints_msg = msg

    def traffic_lights_cb(self, msg):
        """Callback for /vehicle/traffic_lights topic"""
        self.traffic_lights_msg = msg

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, idx, velocity):
        waypoints[idx].twist.twist.linear.x = velocity

    def calc_distances(self, preceding_waypoint, waypoints):
        total_dist = 0
        distances = []

        def dl(a, b):
            return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

        total_dist += dl(preceding_waypoint.pose.pose.position,
                         waypoints[0].pose.pose.position)
        distances.append(total_dist)

        for wp1, wp2 in zip(waypoints, waypoints[1:]):
            total_dist += dl(wp1.pose.pose.position, wp2.pose.pose.position)
            distances.append(total_dist)
        return distances


class WaypointSearch(object):
    """Organize waypoints to make it fast to search for the closest to a position"""
    def __init__(self, waypoints):
        # Preprocess waypoints using the k-d tree algorithm
        self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_idx_ahead(self, x, y):
        closest_idx = self.get_closest_waypoint_idx(x, y)

        if not self.__is_closest_idx_ahead(closest_idx, x, y):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def get_closest_waypoint_idx_behind(self, x, y):
        closest_idx = self.get_closest_waypoint_idx(x, y)
        if self.__is_closest_idx_ahead(closest_idx, x, y):
            closest_idx = (closest_idx - 1) % len(self.waypoints_2d)
        return closest_idx

    def get_closest_waypoint_idx(self, x, y):
        return self.waypoints_tree.query([x, y], 1)[1]

    def __is_closest_idx_ahead(self, closest_idx, x, y):
        """ Check if closest_idx is ahead or behind position"""
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        return val < 0


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
