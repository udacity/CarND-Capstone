#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

# since we have a max speed around 11 mps, with 3s prediction is more than enough for collision avoidance
LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.waypoints = None

        self.speed_limit = rospy.get_param("~speed_limit")

        # let's try to run it a bit faster for more accurate control
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_next_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.base_waypoints_sub.unregister()

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

    def publish_next_waypoints(self):
        waypoints = []

        if self.current_pose and self.waypoints:
            waypoint_begin_index = self.get_next_waypoint_index()
            waypoint_end_index = waypoint_begin_index + LOOKAHEAD_WPS

            if waypoint_end_index > len(self.waypoints):
                waypoint_end_index = len(self.waypoints)

            for i in range(waypoint_begin_index, waypoint_end_index):
                self.set_waypoint_velocity(self.waypoints, i, self.speed_limit)
                waypoints.append(self.waypoints[i])

        output = Lane()
        output.waypoints = waypoints

        self.final_waypoints_pub.publish(output)

    def get_nearest_waypoint_index(self):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        nearest_dist = 99999
        nearest_waypoint_index = 0
        next_waypoint_index = 0

        #dist = dl(self.waypoints[nearest_waypoint_index].pose.pose.position, self.current_pose.position)

        while next_waypoint_index < len(self.waypoints):
            dist = dl(self.waypoints[next_waypoint_index].pose.pose.position, self.current_pose.position)
            if (dist < nearest_dist):
                nearest_dist = dist
                nearest_waypoint_index = next_waypoint_index
            next_waypoint_index += 1

        return nearest_waypoint_index

    def get_next_waypoint_index(self):
        nearest_waypoint_index = self.get_nearest_waypoint_index()
        ahead_waypoint_index = nearest_waypoint_index + 3

        # calculate vectors (current.x - nearest.x, current.y - nearest.y), (current.x - next.x, current.y - next.y)
        # to decide if nearest is the next point or one more
        vec_nearest = np.array([self.current_pose.position.x - self.waypoints[nearest_waypoint_index].pose.pose.position.x, self.current_pose.position.y - self.waypoints[nearest_waypoint_index].pose.pose.position.y])
        vec_next = np.array([self.current_pose.position.x - self.waypoints[ahead_waypoint_index].pose.pose.position.x, self.current_pose.position.y - self.waypoints[ahead_waypoint_index].pose.pose.position.y])

        next_waypoint_index = nearest_waypoint_index
        if (vec_nearest.dot(vec_next) < 0):
            next_waypoint_index = next_waypoint_index + 1

        # quaternion = (
        #     self.current_pose.orientation.x,
        #     self.current_pose.orientation.y,
        #     self.current_pose.orientation.z,
        #     self.current_pose.orientation.w,
        # )
        # #euler = tf.transformations.euler_from_quaternion(quaternion)
        #if (euler[2] > (math.pi/4)):
        #    next_waypoint_index += 1

        return next_waypoint_index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
