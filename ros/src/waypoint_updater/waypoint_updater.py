#!/usr/bin/env python

import math
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
CONSTANT_DECEL = 1 / LOOKAHEAD_WPS  # Deceleration constant for smoother braking
PUBLISHING_RATE = 20  # Rate (Hz) of waypoint publishing
STOP_LINE_MARGIN = 4  # Distance in waypoints to pad in front of the stop line
MAX_DECEL = 0.5
LOGGING_THROTTLE_FACTOR = PUBLISHING_RATE * 2  # Only log at this rate (1 / Hz)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.decelerate_count = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        # rospy.logwarn("closest_idx={}".format(closest_idx))
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            # Distance includes a number of waypoints back so front of car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - STOP_LINE_MARGIN, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist) + (i * CONSTANT_DECEL)
            if vel < 1.0:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        self.decelerate_count += 1
        if (self.decelerate_count % LOGGING_THROTTLE_FACTOR) == 0:
            size = len(waypoints) - 1
            vel_start = temp[0].twist.twist.linear.x
            vel_end = temp[size].twist.twist.linear.x
            rospy.logwarn("DECEL: vel[0]={:.2f}, vel[{}]={:.2f}".format(vel_start, size, vel_end))
        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        if self.stopline_wp_idx != msg.data:
            rospy.logwarn(
                "LIGHT: new stopline_wp_idx={}, old stopline_wp_idx={}".format(msg.data, self.stopline_wp_idx))
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
        dl = lambda a, b: math.sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2))
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
