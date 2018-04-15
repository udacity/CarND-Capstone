#!/usr/bin/env python

import rospy
import numpy as np
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5 # Maximum deceleratiokn rate in m / s^2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = []
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.closest_idx = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():

            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_id(self):
        # Get car position
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Get closest waypoint from tree generated using KDTree
        self.closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Is found point ahead or behinf car
        closest_coord = self.waypoints_2d[self.closest_idx] # Closest found coord
        previous_coord = self.waypoints_2d[self.closest_idx - 1] # previous coord

        closest_vect = np.array(closest_coord)
        previous_vect = np.array(previous_coord)
        position_vect = np.array([x, y])

        val = np.dot(closest_vect - previous_vect, position_vect - closest_vect)

        if val > 0:
            self.closest_idx = (self.closest_idx + 1) % len(self.waypoints_2d)

    def decelerate_wp(self, waypoints):
        processed_wp_buffer = []

        for i, wp in enumerate(waypoints):

            cur_wp = Waypoint()
            cur_wp.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - self.closest_idx - 4, 0) # 4 os conservative wp back from target line 
                                                                      # to prevent car front to passs the line

            distance = self.distance(waypoints, i, stop_idx)

            calc_vel = 0.2 * distance # Used a linear decelaration with smooth rate

            if calc_vel < 1.0: # If velocity is too slow, just make it 0 to avoid endless deceleration
                calc_vel = 0.0

            cur_wp.twist.twist.linear.x = min(calc_vel, wp.twist.twist.linear.x)

            # self.monitor_values(i, calc_vel, wp.twist.twist.linear.x, distance, cur_wp.twist.twist.linear.x)

            processed_wp_buffer.append(cur_wp)

        return processed_wp_buffer

    def monitor_values(self, i, calc_vel, max_vel, distance, final_vel):
        if i == 0 or i == 50 or i == 100 or i == 150 or i == 199:
            print("i:")
            print(i)
            print("calc_vel:")
            print(calc_vel)
            print("Max wp vel:")
            print(max_vel)
            print("distance:")
            print(distance)
            print("closest_idx:")
            print(self.closest_idx)
            print("stopline_wp_idx:")
            print(self.stopline_wp_idx)
            print("final_vel:")
            print(final_vel)


    def generate_lane(self):
        lane = Lane()

        self.get_closest_waypoint_id()
        farmost_idx = self.closest_idx + LOOKAHEAD_WPS

        cur_base_waypoints = self.base_waypoints.waypoints[self.closest_idx:farmost_idx]

        # self.stopline_wp_idx = 500 # Value to test until TL detection is in place

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farmost_idx):
            lane.waypoints = cur_base_waypoints
        else:
            lane.waypoints = self.decelerate_wp(cur_base_waypoints)

        return lane

    def publish_waypoints(self):
        self.final_waypoints_pub.publish( self.generate_lane() )

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

        if len(self.waypoints_2d) == 0:
            for i in range(1, len(waypoints.waypoints)):
                x = waypoints.waypoints[i].pose.pose.position.x
                y = waypoints.waypoints[i].pose.pose.position.y
                self.waypoints_2d.append([x, y])
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # print(msg)
        self.stopline_wp_idx = msg

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
