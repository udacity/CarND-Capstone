#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from scipy.spatial import KDTree
import math
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber(
        #     '/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.base_lane = None
        self.pose = None
        self.base_waypoints = None
        self.traffic_waypoints = None
        self.obstacle_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        rospy.loginfo('Starting WaypointUpdater Node')

        self.loop()

    def initialized_waypoints(self):
        return None not in (
            self.base_lane,
            self.base_waypoints,
            self.waypoints_2d,
            self.waypoint_tree)

    def loop(self):
        # As recommmended, take control of update frequency
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose is not None and self.initialized_waypoints():
                closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints(closest_waypoint_index)
            rate.sleep()

    def get_closest_waypoint_index(self):
        position = self.pose.pose.position
        x, y = position.x, position.y
        closest_index = self.waypoint_tree.query([x, y], 1)[1]

        # check that the waypoint is ahead of the vehicle
        closest_coord = self.waypoints_2d[closest_index]
        prev_coord = self.waypoints_2d[closest_index - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)

        rospy.loginfo('Closest waypoint index = {}'.format(closest_index))

        return closest_index

    def publish_waypoints(self, closest_waypoint_index):
    	final_lane = self.generate_lane()
    	self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
    	lane = Lane()
    	closest_idx = self.get_closest_waypoint_index()
    	farthest_idx = closest_idx + LOOKAHEAD_WPS
    	base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

    	if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
    	    lane.waypoints = base_waypoints
    	else:
    	    lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

    	return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
    	temp = []
    	for i, wp in enumerate(waypoints):
    	    p = Waypoint()
    	    p.pose = wp.pose
    	    stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
    	    dist = self.distance(waypoints, i, stop_idx)
    	    vel = math.sqrt(2 * MAX_DECEL * dist)
    	    if vel <1.:
    		vel = 0

    	    p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    	    temp.append(p)
    	return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        def position(waypoint):
            position = waypoint.pose.pose.position
            return [position.x, position.y]

        if not self.initialized_waypoints():
            self.base_lane = waypoints
            self.base_waypoints = waypoints
            self.waypoints_2d = [ \
                position(waypoint) for waypoint in waypoints.waypoints]
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
        dl = lambda a, b: math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            wp1_pos = waypoints[wp1].pose.pose.position
            i_pos = waypoints[i].pose.pose.position
            dist += dl(wp1wp1_pos, i_pos)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
