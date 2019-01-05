#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math
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
# TODO Check if this is ok
PUBLISH_RATE = 10


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.current_pose = None
        self.current_velocity = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None

        # Saves the subscriber so that it can unregister in its callback
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO Might be useful?
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()
    
    def loop(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints_tree:
                
                closest_idx = self.closest_waypoint_idx()
                final_waypoints = self.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

                # TODO Check traffic light state

                lane = Lane()
                lane.waypoints = final_waypoints

                self.final_waypoints_pub.publish(lane)

            rate.sleep()


    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def velocity_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.current_velocity = math.sqrt(vx**2 + vy**2)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

        # Unsubscribe as we do not need the base waypoints anymore
        self.base_waypoints_sub.unregister()

        rospy.loginfo("Base waypoints data processed, unsubscribed from /base_waypoints")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def closest_waypoint_idx(self):
        ego_position = [self.current_pose.position.x, self.current_pose.position.y]

        closest_idx = self.waypoints_tree.query(ego_position, 1)[1]

        closest_waypoint = np.array(self.waypoints_2d[closest_idx])
        previous_waypoint = np.array(self.waypoints_2d[closest_idx - 1])

        ego_position = np.array(ego_position)

        waypoint_vec = closest_waypoint - previous_waypoint
        ego_vec = ego_position - closest_waypoint

        # Computes direction
        val = np.dot(waypoint_vec, ego_vec)

        # Checks if the waypoint is behind
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx


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
