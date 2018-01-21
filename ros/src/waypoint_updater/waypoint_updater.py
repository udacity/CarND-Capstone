#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.map_index = 0
        self.last_distance = sys.float_info.max
        rospy.spin()

    def pose_distance(self, waypoint): 
        a=waypoint.pose.pose.position
        b=self.pose.pose.position
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    
    def set_closest_mapindex(self):
        distance = sys.float_info.max
        leastd_index = self.map_index
        base_lane_len = len(self.base_lane.waypoints)
        for index in range(0, base_lane_len):
            tindex = ((self.map_index + index) % base_lane_len)
            wayp = self.base_lane.waypoints[tindex]
            newdist = self.pose_distance(wayp)
            if distance > newdist:
                distance = newdist
                leastd_index = tindex
            else:
                break

        # check if leastd_index or leastd_index-1
        rospy.loginfo("i: " + str(leastd_index) + " d: " + str(distance) )
        self.map_index = leastd_index

    def pose_cb(self, msg):
        self.pose = msg
        if self.base_lane is None:
            return
        self.set_closest_mapindex()
        # create lane object
        lane_op = Lane()
        # copy waypoints ahead of pose and pass it to lane_op
        base_lane_len = len(self.base_lane.waypoints)
        if self.map_index + LOOKAHEAD_WPS < base_lane_len:
            lane_op.waypoints = self.base_lane.waypoints[self.map_index:self.map_index+LOOKAHEAD_WPS]
        else:
            start = self.map_index
            end = (self.map_index + LOOKAHEAD_WPS) % base_lane_len
            lane_op.waypoints = self.base_lane.waypoints[start:] + self.base_lane.waypoints[0:end]
        # publish lane object
        self.final_waypoints_pub.publish(lane_op)

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints

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
