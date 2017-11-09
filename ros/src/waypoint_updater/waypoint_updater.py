#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        ego_q = msg.orientation
        ego_direction_3d = inverse(ego_q) * Quaternion(1,0,0,0) * ego_q
        ego_vx = np.array([ego_direction_3d.x,ego_direction_3d,y])
        ego_vy = np.array([[0,-1],[1,0]]) * ego_vx

        try:
            waypoints

            # publish final_waypoints
            min_distance = 1e9
            initial_wp_index = 0
            for i in xrange(len(waypoints)):
                    waypoint = waypoints[i].pose.pose.position
                    np_waypoint = np.array([waypoint.x,waypoint.y])
                    np_ego_pose = np.array([msg.position.x,msg.position.y])
                    np_diff = np_waypoint - np_ego_pose
                    distance = np.linalg.norm(np_diff)
                    param = np.linalg.inv(np.concatenate((ego_vx,ego_vy),axis=0)) * np_diff

                    # If the waypoint is in front of vehicle and also the closest one,
                    # update the index.
                    if(param[0] > 0 and distance < min_distance):
                        min_distance = distance
                        initial_wp_index = i % len(waypoints)

            # publish
            final_waypoints = []
            for i in xrange(len(LOOKAHEAD_WPS)):
                index = (initial_wp_index + i) % len(waypoints)
                final_waypoints.append(waypoints[index])
            final_waypoints_pub(final_waypoints)

        except NameError:
            # do nothing

        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic = msg
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
