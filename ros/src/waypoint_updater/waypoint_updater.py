#!/usr/bin/env python
from __future__ import division

import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from visualization_msgs.msg import Marker

'''
This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which
does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status
of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node as well as to
verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we publish
MIN_VEL = 1.


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.max_dec = 0.8 * abs(rospy.get_param('/dbw_node/decel_limit'))

        self.waypoints = None
        self.n_waypoints = 0
        self.ego = None
        self.next_idx = -1
        self.tl_idx = -1

        # ROS publishers
        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)


        # ROS subscribers
        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane,
                         self.waypoints_cb, queue_size=1)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.next_idx = self.find_next_waypoint()
        if -1 < self.next_idx and not rospy.is_shutdown():

            rospy.loginfo("Current position ({}, {}), next waypoint: {}"
                          .format(self.ego.pose.position.x,
                                  self.ego.pose.position.y,
                                  self.next_idx))

            waypoints = list(self.get_lookahead(self.next_idx))

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = waypoints
            self.pub.publish(lane)

    def get_lookahead(self, start_idx):
        for i in range(LOOKAHEAD_WPS):
            idx = (start_idx + i + self.n_waypoints) % self.n_waypoints
            yield copy.deepcopy(self.waypoints[idx])

    def pose_cb(self, pose):
        if self.ego is None or self.ego.header.seq < pose.header.seq:
            self.ego = pose

    def waypoints_cb(self, lane):
        if self.waypoints is None and lane.waypoints is not None:
            self.waypoints = lane.waypoints
            self.n_waypoints = len(lane.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass
    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    @classmethod
    def vector_from_quaternion(cls, q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def find_next_waypoint(self):
        min_dist = 1e10
        min_idx = -1
        if self.ego and self.waypoints:
            ego_pose = self.ego.pose
            # Find the closest waypoint
            for i in range(self.next_idx, self.next_idx + self.n_waypoints):
                idx = (i + self.n_waypoints) % self.n_waypoints
                wp_pos = self.waypoints[idx].pose.pose.position
                dl = self.distance(ego_pose.position, wp_pos)
                if dl < min_dist:
                    min_dist = dl
                    min_idx = idx
                if min_dist < 10 and dl > min_dist:
                    break

            # Check if we are behind or past the closest waypoint
            wp_pos = self.waypoints[min_idx].pose.pose.position
            pos = copy.deepcopy(ego_pose.position)
            x, y, z = self.vector_from_quaternion(ego_pose.orientation)
            pos.x += x * .1
            pos.y += y * .1
            pos.z += z * .1
            if self.distance(wp_pos, pos) > min_dist:
                min_idx = (min_idx + 1) % self.n_waypoints
        return min_idx

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 +(a.y-b.y)**2 +(a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[(i)].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')