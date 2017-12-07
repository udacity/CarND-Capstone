#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
from operator import itemgetter

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO: Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    dl = lambda self, a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
#       rospy.Subscriber('/obstacle_waypoint', ?, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.waypoints_count = 0
        self.pose = None
        self.poses = 0

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose

        # Process if the waypoints list exists (from /base_waypoints)
        if self.waypoints:
            # Find the closest waypoint (TODO: Needs to be the 'ahead' waypoint)
            distances = [self.dl(self.pose.position, waypoint.pose.pose.position) for waypoint in self.waypoints[:self.waypoints_count]]
            nearest_waypoint = min(enumerate(distances), key=itemgetter(1))[0]
            final_waypoints = self.waypoints[nearest_waypoint:nearest_waypoint+LOOKAHEAD_WPS+1]
            pub = Lane()
            pub.header = msg.header
            pub.waypoints = final_waypoints
            self.final_waypoints_pub.publish(pub)
            self.poses += 1
            if self.poses%50 == 0:
                rospy.logwarn("{} poses parsed (nearest_waypoint={}).".format(self.poses, nearest_waypoint))
        else:
            rospy.logerr("/current_pose received before /base_waypoints.")

    def waypoints_cb(self, waypoints):
        # /base_waypoints should only be passed once at initialisation
        if self.waypoints:
            rospy.logerr("/base_waypoints message received multiple times!")
        self.waypoints = waypoints.waypoints
        self.waypoints_count = len(self.waypoints)
        # Extend waypoints to include the 'wraparound' required for a circular track
        # as a once-off process here.
        self.waypoints += self.waypoints[:LOOKAHEAD_WPS]
        rospy.logwarn("{} waypoints received from /base_waypoints.".format(self.waypoints_count))

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
        for i in range(wp1, wp2+1):
            dist += self.dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')