#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_pos = None
        self.base_waypoints = None
        self.last_wp = None
        self.frame_id = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.last_pos = msg.pose
        self.frame_id = msg.header.frame_id

        if self.base_waypoints is not None:
            # get index closest to current position
            self.last_wp = self.nearest_wp(self.last_pos.position, self.base_waypoints)+1
            # fetch next LOOKAHEAD number of waypoints
            ahead = min(len(self.base_waypoints),self.last_wp+LOOKAHEAD_WPS)
            lookAheadWpts = self.base_waypoints[self.last_wp:ahead]
            # construct speed for lookAheadWpts
            for waypoint in lookAheadWpts:
                waypoint.twist.twist.linear.x = 4.4
            # construct message to be sent
            message_to_sent = Lane()
            message_to_sent.header.stamp = rospy.Time.now()
            message_to_sent.header.frame_id = self.frame_id
            message_to_sent.waypoints = lookAheadWpts
            rospy.logwarn("The last position is: %s", self.last_pos.position)
            rospy.logwarn("The last wp_index is: %d", self.last_wp)
            self.final_waypoints_pub.publish(message_to_sent)
       
    def nearest_wp(self, last_position, waypoints):
        """find nearest waypoint index to the current location"""
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        nearest_distance = 9999;
        nearest_index = -1;
        for index, waypoint in enumerate(waypoints):
            waypoint_pos = waypoint.pose.pose.position
            distance = dl(last_position, waypoint_pos)
            if distance < nearest_distance:
                nearest_index = index
                nearest_distance = distance
        return nearest_index

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        """Store the map data"""
        self.base_waypoints = waypoints.waypoints

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
