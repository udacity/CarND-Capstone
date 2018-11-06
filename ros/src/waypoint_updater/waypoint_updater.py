#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
UPDATE_RATE = 50 #hz

class WaypointUpdater(object):
    def __init__(self, rate_hz=UPDATE_RATE):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_ktree = None
        self.freq = rate_hz
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_ktree != None:
                nearest_wp_indx = self.get_nearest_wp_indx()
                self.publish_waypoints(nearest_wp_indx)
            rate.sleep()

    def publish_waypoints(self, nearest_indx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[nearest_indx:nearest_indx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def get_nearest_wp_indx(self):
        ptx = self.pose.pose.position.x
        pty = self.pose.pose.position.y
        nearest_indx = self.waypoint_ktree.query([ptx,pty],1)[1]

        nearest_coord = self.waypoints_2d[nearest_indx]
        prev_coord = self.waypoints_2d[nearest_indx - 1]

        neareset_vect = np.array(nearest_coord)
        prev_vect = np.array(prev_coord)
        positive_vect = np.array([ptx,pty])

        val = np.dot(neareset_vect-prev_vect, positive_vect-neareset_vect)

        if val > 0:
            nearest_indx = (nearest_indx + 1) % len(self.waypoints_2d)

        return nearest_indx

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.base_waypoints = lane
        if not self.waypoints_2d:
            self.waypoints_2d = [ [ waypoint.pose.pose.position.x, waypoint.pose.pose.position.y ] for waypoint in lane.waypoints ]
            self.waypoint_ktree = KDTree(self.waypoints_2d)

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
