#!/usr/bin/env python
import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from scipy.spatial import KDTree
import numpy as np

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

LOOKAHEAD_WPS = 45  # Number of waypoints we publish

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # ROS subscribers
        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane,
                         self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # ROS publishers
        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.ego = None
        self.next_idx = -1

        self.publishing_loop()

    def publishing_loop(self):
        '''
        Main loop finding the next LOOKAHEAD_WPS waypoints and publishing
        them in the final_waypoints topic.
        :return: None
        '''
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.ego and self.base_waypoints:
                closest_waypoint_idx = self.find_next_waypoint()
                rospy.loginfo("Current position ({}, {}), next waypoint position: ({}, {})"
                              .format(self.ego.pose.position.x,
                                      self.ego.pose.position.y,
                                      self.waypoints_2d[closest_waypoint_idx][0],
                                      self.waypoints_2d[closest_waypoint_idx][1],))
                self.publish(closest_waypoint_idx)
            rate.sleep()

    def pose_cb(self, pose):
        #Get current pose
        self.ego = pose

    def waypoints_cb(self, waypoints):
        '''
        Get list of waypoints along our lane and build a KD Tree
        :param waypoints:
        '''
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        #Get desired waypoint velocity
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        #Set a waypoints desired velocity
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        #Calculate distance between waypoints from the list of waypoints
        dist = 0
        for i in range(wp1, wp2+1):
            dist += euclidean_dist(waypoints[wp1].pose.pose.position, waypoints[(i)].pose.pose.position)
            wp1 = i
        return dist

    def euclidean_dist(self, pt1, pt2):
        """
        Return the Euclidean distance between two points
        :pt1: geometry_msgs/Point
        :pt2: geometry_msgs/Point
        """
        return math.sqrt((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2 +
                         (pt1.z - pt2.z) ** 2)

    def publish(self, closest_waypoint):
        '''
        Publish list of next set of waypoints the car will be to reach
        :return: None
        '''
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.base_waypoints.waypoints[closest_waypoint:closest_waypoint + LOOKAHEAD_WPS]
        self.pub.publish(lane)

    def vector_from_quaternion(self, q):
        #Used to convert to vector coord from quaternion for ego vehicle coord
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def find_next_waypoint(self):
        '''
        Finds the next closest waypoint in front of the vehicle
        :return: Index of the closest waypoint in the waypoint_2d array
        '''
        x = self.ego.pose.position.x
        y = self.ego.pose.position.y
        # This returned index is matched onto the indexes of self.waypoints_2d
        closest_waypoint_index = self.waypoints_tree.query([x,y], 1)[1]

        # Since we want to only return waypoints ahead of the vehicle
        # We test whether the waypoint is ahead or behind of the current
        # position of the car.
        closest_coord = self.waypoints_2d[closest_waypoint_index]
        previous_closest_coord = self.waypoints_2d[closest_waypoint_index-1]

        closest_coord_vec = np.array(closest_coord)
        prev_vec = np.array(previous_closest_coord)
        position_vec = np.array([x, y])

        val = np.dot(closest_coord_vec-prev_vec, position_vec-closest_coord_vec)

        # If the dot product is positive, the car is ahead of the closest waypoint found.
        # Then we take the next one in the list.
        if val > 0:
            closest_waypoint_index = (closest_waypoint_index + 1) % len(self.waypoints_2d)
        return closest_waypoint_index

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')