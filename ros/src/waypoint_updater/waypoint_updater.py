#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
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
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
UPDATE_RATE = 30 #hz
NO_WP = -1
DECEL_RATE = 4.9 # m/s^2
STOPLINE = 3 # waypoints behind stopline to stop

class WaypointUpdater(object):
    def __init__(self, rate_hz=UPDATE_RATE):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_ktree = None
        self.freq = rate_hz
        self.nearest_wp_idx = NO_WP
        self.stop_wp = NO_WP
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_ktree != None:
                self.nearest_wp_idx = self.get_nearest_wp_indx()
                self.publish_waypoints()
                # don't update unless we get new positional data
                self.pose = None
            rate.sleep()

    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header
        look_ahead_wp_max = self.nearest_wp_idx + LOOKAHEAD_WPS
        base_wpts = self.base_waypoints.waypoints[self.nearest_wp_idx:look_ahead_wp_max]
        if self.stop_wp == NO_WP or (self.stop_wp >= look_ahead_wp_max):
            lane.waypoints = base_wpts
        else:
            temp_waypoints = []
            stop_idx = max(self.stop_wp - self.nearest_wp_idx - STOPLINE, 0)
            for i, wp in enumerate(base_wpts):
                temp_wp = Waypoint()
                temp_wp.pose = wp.pose
                if stop_idx >= STOPLINE:
                    dist = self.distance(base_wpts, i, stop_idx)
                    # v^2 = vo^2 + 2*a*(x-xo)
                    # v^2 = 0 + 2*a*(dist)
                    # v = sqrt(2*a*dist)
                    vel = math.sqrt(2*DECEL_RATE*dist)
                    if vel < 1.0:
                        vel = 0.0
                else:
                    vel = 0.0
                temp_wp.twist.twist.linear.x = min(vel, base_wpts[0].twist.twist.linear.x)
                temp_waypoints.append(temp_wp)
            lane.waypoints = temp_waypoints
        return lane

    def get_nearest_wp_indx(self):
        ptx = self.pose.pose.position.x
        pty = self.pose.pose.position.y
        nearest_indx = self.waypoint_ktree.query([ptx,pty],1)[1]

        nearest_coord = self.waypoints_2d[nearest_indx]
        prev_coord = self.waypoints_2d[nearest_indx - 1]

        neareset_vect = np.array(nearest_coord)
        prev_vect = np.array(prev_coord)
        positive_vect = np.array([ptx,pty])
        
        # check if the nearest_coord is infront or behind the car
        val = np.dot(neareset_vect-prev_vect, positive_vect-neareset_vect)

        if val > 0.0:
            # works for waypoints that are in a loop
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
        self.stop_wp = msg.data

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
