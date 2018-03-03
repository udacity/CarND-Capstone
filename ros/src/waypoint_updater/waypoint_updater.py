#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial.distance import cdist
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo("Gauss - Started WaypointUpdater")

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # @done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # Removed for now, as those raise warnings
        # rospy.Subscriber('/traffic_waypoint', Waypoint, self.pose_cb)
        # rospy.Subscriber('/obstacle_waypoint', Waypoint, self.pose_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # @done: Add other member variables you need below
        self.base_waypoints_msg = None

        rospy.spin()

    def pose_cb(self, msg):
        # @done: Implement
        x = msg.pose.position.x
        y = msg.pose.position.y
        rospy.loginfo("Gauss - Got pose (x, y): " + str(x) + ", " + str(y))

        if self.base_waypoints_msg is not None:
            waypoints = self.base_waypoints_msg.waypoints

            index = self.closest_waypoint_index(msg.pose.position, waypoints)
            waypoints_sliced = waypoints[index:index+LOOKAHEAD_WPS]

            output_msg = Lane()
            output_msg.header = self.base_waypoints_msg.header
            output_msg.waypoints = waypoints_sliced

            rospy.loginfo("Gauss - Publishing waypoints of length: " + str(len(output_msg.waypoints)))
            self.final_waypoints_pub.publish(output_msg)

    def waypoints_cb(self, waypoints):
        # @done: Implement
        rospy.loginfo("Gauss - Got Waypoints")
        self.base_waypoints_msg = waypoints

    def closest_waypoint_index(self, position, waypoints):
        positions = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints]
        return cdist([[position.x, position.y]], positions).argmin()

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

