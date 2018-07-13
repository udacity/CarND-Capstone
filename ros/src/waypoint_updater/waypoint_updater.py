#!/usr/bin/env python

import rospy
from copy import deepcopy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLight, Waypoint
import numpy as np
import math
import yaml
from speed_calculator import SpeedCalculator
from waypoint_search import WaypointSearch
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
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # ROS topic messages
        self.pose_msg = None
        self.velocity_msg = None
        self.max_velocity = None
        self.base_waypoints_msg = None
        self.traffic_waypoint_msg = None
        self.dbw_enabled_msg = None

        self.rate = rospy.Rate(50)

        self.waitUntilInit()
        self.loopForEver()

    def waitUntilInit(self):
        """Wait until all subscriptions has provided at least one msg."""
        while not rospy.is_shutdown():
            if None not in (self.pose_msg, self.velocity_msg, self.base_waypoints_msg,
                            self.traffic_waypoint_msg, self.dbw_enabled_msg):
                self.wp_calc = WaypointCalculator(self.base_waypoints_msg)
                break
            self.rate.sleep()

    def loopForEver(self):
        while not rospy.is_shutdown():
            if self.dbw_enabled_msg.data:
                waypoints = self.wp_calc.calc_waypoints(self.pose_msg, self.velocity_msg, self.traffic_waypoint_msg)
                self.publish_waypoints(waypoints)
            else:
                self.wp_calc.reset()
            self.rate.sleep()

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

    def traffic_waypoint_cb(self, msg):
        """Callback for /traffic_waypoint topic"""
        self.traffic_waypoint_msg = msg

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled_msg = msg


class WaypointCalculator(object):
    def __init__(self, base_waypoints_msg):
        self.base_waypoints_msg = base_waypoints_msg
        self.previous_first_idx = None
        self.waypoints = None
        self.wp_search = WaypointSearch(self.base_waypoints_msg.waypoints)
        self.__set_max_velocity()

    def reset(self):
        """Reset internal state.

        Call this method when drive-by-wire is reactivated after manual control.
        """
        # Let preceding_waypoint be calculated from current vehicle state, instead of using some out-dated index.
        self.previous_first_idx = None

    def __set_max_velocity(self):
        """Set the max_velocity according to velocities in the base waypoints.

          All base waypoints should have the same velocity (in this project). This assumption is also verified to be
          correct.
        """
        self.max_velocity = get_waypoint_velocity(self.base_waypoints_msg.waypoints[0])
        assert(all(self.max_velocity == get_waypoint_velocity(waypoint)
                   for waypoint in self.base_waypoints_msg.waypoints))

    def calc_waypoints(self, pose_msg, velocity_msg, traffic_waypoint_msg):
        # Extract base_waypoints ahead of vehicle as a starting point for the final_waypoints.
        first_idx = self.wp_search.get_closest_waypoint_idx_ahead(pose_msg.pose.position.x,
                                                                  pose_msg.pose.position.y)
        if self.previous_first_idx is None:
            # Construct previous waypoint from the vehicles initial pose and velocity.
            self.preceding_waypoint = Waypoint()
            self.preceding_waypoint.pose.pose = pose_msg.pose
            self.preceding_waypoint.twist.twist = velocity_msg.twist
            self.preceding_waypoint.acceleration = 0.0
        elif self.previous_first_idx != first_idx:
            # Copy the preceding waypoint before updating the waypoint list.
            self.preceding_waypoint = self.waypoints[first_idx - self.previous_first_idx - 1]

        self.waypoints = deepcopy(self.base_waypoints_msg.waypoints[first_idx:first_idx + LOOKAHEAD_WPS])
        self.previous_first_idx = first_idx

        # Calculate the final_waypoints
        self.__accelerate_to_speed_limit()
        self.__adjust_for_traffic_lights(first_idx, traffic_waypoint_msg)
        self.__assert_speed_limit()
        return self.waypoints

    def __accelerate_to_speed_limit(self):
        """Set waypoints speed to accelerate the vehicle to max velocity."""

        # Base the acceleration on the velocity from preceding waypoint.
        current_speed = get_waypoint_velocity(self.preceding_waypoint)
        current_acceleration = self.preceding_waypoint.acceleration
        speed_calc = SpeedCalculator(target_speed=self.max_velocity, current_speed=current_speed,
                                     target_acceleration=0.0, current_accleration=current_acceleration,
                                     acceleration_limit=10.0, jerk_limit=10.0)
        distances = self.__calc_distances()
        for idx in range(len(self.waypoints)):
            speed = speed_calc.get_speed_at_distance(distances[idx])
            set_waypoint_velocity(self.waypoints, idx, speed)
            acceleration = speed_calc.get_acceleration_at_distance(distances[idx])
            self.waypoints[idx].acceleration = acceleration

    def __calc_distances(self):
        """Calculates the distances from the preceding waypoint to each of the waypoints in the path."""
        total_dist = 0
        distances = []

        def dl(a, b):
            return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

        total_dist += dl(self.preceding_waypoint.pose.pose.position,
                         self.waypoints[0].pose.pose.position)
        distances.append(total_dist)

        for wp1, wp2 in zip(self.waypoints, self.waypoints[1:]):
            total_dist += dl(wp1.pose.pose.position, wp2.pose.pose.position)
            distances.append(total_dist)
        return distances

    def __adjust_for_traffic_lights(self, first_idx, traffic_waypoint_msg):
        """Adjust the speed to break for traffic lights ahead that not is at state green.

        Parameters
        ----------
        first_idx : The base_waypoint index from where waypoints have been derived.
        traffic_waypoint_msg : contains traffic-light waypoint to stop at, otherwise -1
        """

        if traffic_waypoint_msg.data is not -1:
            stop_idx = (traffic_waypoint_msg.data - first_idx) % len(self.base_waypoints_msg.waypoints)
            stop_idx_with_margin = max(0, stop_idx - STOPLINE_WPS_MARGIN)

            if stop_idx_with_margin < len(self.waypoints):
                rospy.loginfo("Stopping at base_idx=%s final_idx=%s", traffic_waypoint_msg.data, stop_idx_with_margin)
                self.__stop_at_waypoint(stop_idx_with_margin)
            else:
                rospy.loginfo("Stopping at base_idx=%s final_idx=%s (beyond planed trajectory)",
                              traffic_waypoint_msg.data, stop_idx_with_margin)
        else:
            rospy.loginfo("No speed adjustments for traffic lights.")

    def __stop_at_waypoint(self, stop_idx):
        """ Reduce speed to stop at stop_idx.

        Parameters
        ----------
        stop_idx : The waypoint index where the vehicle should reach a speed of zero.
        """

        stop_idx += 1  # Offset for preceding waypoint

        # Calculate the reverse distances from the stop_idx and backwards.
        distances = [0.0] + self.__calc_distances()  # distance to preceding waypoint is 0
        distances = [distances[stop_idx] - distance for distance in distances]

        # Try more and more harsh jerk_limits until finding one that decelerate to stop in time.
        # When also unable to stop using the max permitted jerk limit of 10 m/s^3, then deceleration is skipped.
        for jerk_limit in np.arange(2.5, 10.1, 2.5):
            # Create temporary lists with speed and acceleration to be modified below.
            speeds = [get_waypoint_velocity(self.preceding_waypoint)] + \
                     [get_waypoint_velocity(wp) for wp in self.waypoints]
            accs = [self.preceding_waypoint.acceleration] + \
                   [wp.acceleration for wp in self.waypoints]

            # Set the speed and acceleration after the stop_idx to zero.
            speeds[stop_idx + 1:] = [0.0] * (len(speeds) - stop_idx - 1)
            accs[stop_idx + 1:] = [0.0] * (len(accs) - stop_idx - 1)

            # Calculate the deceleration backwards from the stop_idx,
            # until reaching a speed that is greater than what was already requested.
            speed_calc = SpeedCalculator(target_speed=self.max_velocity, current_speed=0.0,
                                         target_acceleration=0.0, current_accleration=0.0,
                                         acceleration_limit=10.0, jerk_limit=jerk_limit)

            for idx in range(stop_idx, -1, -1):
                speed = speed_calc.get_speed_at_distance(distances[idx])
                acc = -speed_calc.get_acceleration_at_distance(distances[idx])
                if speed > speeds[idx] or np.isclose(speed, speeds[idx]):
                    rospy.logdebug('Success: jerk_limit %s decelerates from %s m/s (>=%s m/s) at idx %s',
                                   jerk_limit, speed, speeds[idx], idx)
                    stop_possible = True
                    break
                speeds[idx] = speed
                accs[idx] = acc

            else:
                rospy.logdebug('Failed: jerk_limit %s only decelerates from %s m/s (<%s m/s) at idx %s',
                               jerk_limit, speed, get_waypoint_velocity(self.preceding_waypoint), idx)
                stop_possible = False

            if stop_possible:
                for idx in range(len(self.waypoints)):
                    set_waypoint_velocity(self.waypoints, idx, speeds[idx + 1])
                    self.waypoints[idx].acceleration = accs[idx + 1]
                break
        else:
            rospy.loginfo('Unable to stop from %s m/s in %s m',
                          get_waypoint_velocity(self.preceding_waypoint),
                          distances[0])

    def __assert_speed_limit(self):
        """Makes sure we never exceeds max velocity"""
        for i in range(len(self.waypoints)):
            velocity = get_waypoint_velocity(self.waypoints[i])
            if velocity > self.max_velocity:
                # Note that this is not expected to ever happen. However it's perhaps not a fatal error since the
                # velocity can be limited to the max allowed value here.
                rospy.logerr('Calculated velocity %s exceeds max allowed value %s.', velocity, self.max_velocity)
                set_waypoint_velocity(self.waypoints, i, self.max_velocity)


# Helper functions
def get_waypoint_velocity(waypoint):
    return waypoint.twist.twist.linear.x


def set_waypoint_velocity(waypoints, idx, velocity):
    waypoints[idx].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
