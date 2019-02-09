#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

from scipy.spatial import KDTree
import numpy as np

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
UPDATE_FREQUENCY = 10 # rate in HZ
MAX_DECEL = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Initialize the variables
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic_wp = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(UPDATE_FREQUENCY)
        while not rospy.is_shutdown():
            # If the base waypoints and the current position were received
            if self.pose and self.waypoint_tree:
                # Find the closest waypoint ahead of the current position
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                # Publish the waypoints from the closest waypoint
                self.publish_waypoints(closest_waypoint_idx)
                rospy.loginfo("pos:%d, %d, The closest_idx is %d\n", self.pose.pose.position.x, self.pose.pose.position.y, closest_waypoint_idx)

            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # check if closet_idx is ahead or behind of the current position
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # If val positive, it means the closest point is behind
        # the current position, then we need to use the next waypoint
        # so it's ahead of the current position
        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()                   
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # lane.header = self.base_waypoints.header
        if (self.traffic_wp == -1) or (self.traffic_wp >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

#        self.final_waypoints_pub.publish(lane)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i in range(len(waypoints)): # we could also use: for i, wp in enumerate(waypoints)
            wp = waypoints[i]

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.traffic_wp - closest_idx - 3, 0) # the first waypoint is in the middle of the car. We compensate subtract the number of points from the middle to the front of the car to compensate that distance
            dist = self.distance(waypoints, i, stop_idx)
            velocity = math.sqrt(2 * MAX_DECEL * dist)
            if velocity < 1:
                velocity = 0

            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x) # Keep the speed limit
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # update the current position
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # setup the base waypoints
        self.base_waypoints = waypoints
        # create an array for the waypoints' (x, y) position
        if not self.waypoints_2d:
            self.waypoints_2d = []
            for waypoint in waypoints.waypoints:
                self.waypoints_2d.append([waypoint.pose.pose.position.x, waypoint.pose.pose.position.y])
            # Create a KDTree from the 2D position array
            # KDTree can be used to find the closest waypoint from the current position
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.traffic_wp = msg.data

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
            if (wp1 > len(waypoints)) or (i > len(waypoints)):
                rospy.logwarn("Out of range %d, %d\n", wp1, i)
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
