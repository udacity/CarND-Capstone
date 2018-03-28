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
STOP_AHEAD = 5  # Number of waypoints ahead of TL the car should stop


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
        self.pub_path = rospy.Publisher('/local_path', Marker, queue_size=1)
        self.pub_next = rospy.Publisher('/next_waypoint',
                                        PoseStamped, queue_size=1)
        self.pub_next_tl = rospy.Publisher('/next_tl',
                                           PoseStamped, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane,
                         self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32,
                         self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane,
                         self.obstacle_cb, queue_size=1)

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

            if self.tl_idx > -1:
                self.decelerate(waypoints)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = waypoints
            self.pub.publish(lane)

            # Publish to rviz
            self.publish_next_wp(waypoints[0])
            self.publish_rviz_path(waypoints)

    def publish_rviz_path(self, waypoints):
        path = Marker()
        path.header.frame_id = '/world'
        path.header.stamp = rospy.Time.now()
        path.ns = "path"
        path.id = 0
        path.action = Marker.ADD
        path.type = Marker.LINE_LIST
        path.scale.x = 0.1
        path.color.r = 1.0
        path.color.a = 1.0
        path.points = []
        for wp in waypoints:
            pos = copy.deepcopy(wp.pose.pose.position)
            path.points.append(pos)
            pos = copy.deepcopy(pos)
            pos.z = wp.twist.twist.linear.x
            path.points.append(pos)
        self.pub_path.publish(path)

    def publish_next_wp(self, wp):
        # This is needed for visualising in rviz
        next_pose = copy.deepcopy(wp.pose)
        next_pose.header.frame_id = '/world'
        next_pose.header.stamp = rospy.Time.now()
        self.pub_next.publish(next_pose)

    def publish_next_tl(self, wp):
        # Send the next TL waypoint to rviz
        next_tl = copy.deepcopy(wp.pose)
        next_tl.header.frame_id = '/world'
        next_tl.header.stamp = rospy.Time.now()
        self.pub_next_tl.publish(next_tl)

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
        if msg.data != self.tl_idx:
            self.tl_idx = msg.data
            if self.tl_idx > -1 and self.waypoints is not None:
                self.publish_next_tl(self.waypoints[self.tl_idx])

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
                dl = self.euclidean(ego_pose.position, wp_pos)
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
            if self.euclidean(wp_pos, pos) > min_dist:
                min_idx = (min_idx + 1) % self.n_waypoints
        return min_idx

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @classmethod
    def euclidean(cls, pos1, pos2):
        """
        Return the Euclidean distance between two points

        :param pos1: geometry_msgs/Point
        :param pos2: geometry_msgs/Point
        :return: Euclidean distance between two points
        """
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 +
                         (pos1.z - pos2.z) ** 2)

    @classmethod
    def distance(cls, waypoints, wp1, wp2):
        dl = 0
        n = len(waypoints)
        for i in range(wp1, wp2):
            dl += cls.euclidean(waypoints[i % n].pose.pose.position,
                                waypoints[(i + 1) % n].pose.pose.position)
        return dl

    def dist_to_tl(self):
        if self.tl_idx == -1 or self.next_idx == -1:
            return -1
        diff = self.tl_idx - self.next_idx
        if diff < 0:
            diff += self.n_waypoints
        return diff

    def decelerate(self, waypoints):
        dist = self.dist_to_tl()
        if self.tl_idx > - 1 and dist < LOOKAHEAD_WPS:
            prev_idx = max(dist-STOP_AHEAD, 0)
            prev_wp = waypoints[prev_idx]
            for i in range(prev_idx, LOOKAHEAD_WPS):
                self.set_waypoint_velocity(waypoints, i, 0.)

            for i in range(prev_idx-1, -1, -1):
                wp = waypoints[i]
                d = self.distance(waypoints, i, i + 1)
                prev_vel = self.get_waypoint_velocity(prev_wp)
                vel = math.sqrt(2 * self.max_dec * d + prev_vel**2)
                if vel < MIN_VEL:
                    vel = 0.
                old_vel = self.get_waypoint_velocity(wp)
                if vel > old_vel:
                    break
                self.set_waypoint_velocity(waypoints, i, vel)
                prev_wp = wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')