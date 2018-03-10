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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        self.base_waypoints = None
        self.current_pose = None
        self.waypoints_ahead = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.updater_loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def updater_loop(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            if self.current_pose and self.base_waypoints:
                
                # locate vehicle and update waypoints ahead
                ego_veh_wp           = self.ego_veh_waypoint(self.current_pose,self.base_waypoints)
                self.waypoints_ahead = self.base_waypoints[ego_veh_wp:ego_veh_wp+LOOKAHEAD_WPS]

                # publish
                final_waypoints_msg              = Lane()
                final_waypoints_msg.header.stamp = rospy.Time.now()
                final_waypoints_msg.waypoints    = self.waypoints_ahead
                self.final_waypoints_pub.publish(final_waypoints_msg)

            rate.sleep()


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

    def ego_veh_waypoint(self, pose, waypoints):
        dist_from_current_pose = 100000
        ego_veh_wp_idx = 0
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(waypoints)):
            dist = dl(pose.position, waypoints[i].pose.pose.position)
            if (dist < dist_from_current_pose):
                dist_from_current_pose = dist
                ego_veh_wp_idx = i
        return ego_veh_wp_idx  

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
