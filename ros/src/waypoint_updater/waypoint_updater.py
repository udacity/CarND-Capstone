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
        self.base_waypoints = None
        self.pose = None

        rospy.spin()

    def pose_cb(self, msg):
        """Produce and publish a Lane object containing up to LOOKAHEAD_WPS
        waypoints starting with the closest waypoint that is ahead of the car."""
        rospy.loginfo("waypoint_updater received a position")
        self.pose = msg.pose
        if self.base_waypoints:
            # Get the position and yaw of the car in euler coordinates
            car_x = self.pose.position.x
            car_y = self.pose.position.y
            car_quaternion = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
            car_euler = tf.transformations.euler_from_quaternion(car_quaternion)
            car_yaw = car_euler[2] # Radians, not 100% sure of range
            # Only yaw is considered when answering if a waypoint is ahead of the car.  Even if roll and pitch aren't zero, they should not matter under realistic conditions.

            # Loop through the waypoints to find the closest that is ahead of the car
            closest_point = None
            closest_distance = float('inf')
            for i in range(0, len(self.base_waypoints)):
                waypoint = self.base_waypoints[i]
                if self.is_ahead_of(car_x, car_y, car_yaw, waypoint):
                    distance = math.sqrt((car_x - waypoint.pose.pose.position.x)**2 + (car_y - waypoint.pose.pose.position.y)**2)
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_point = i

            # final_waypoints is the next LOOKAHEAD_WPS waypoints ahead of the car
            final_waypoints = self.base_waypoints[closest_point:closest_point + LOOKAHEAD_WPS]  # final_waypoints will get shorter as the last waypoint is approached
            rospy.loginfo("waypoint_updater closest_point %s, closest_distance %s, len(final_waypoints) %s", closest_point, closest_distance, len(final_waypoints))

            # Publish
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(lane)

        pass

    def is_ahead_of(self, x, y, yaw, waypoint):
        """Helper function to determine if waypoint lies ahead of the car (i.e., within a 180 deg arc centered in the direction of the car).
        This is done through the use of the dot product, dotProduct=|A|*|B|*cos(theta), where theta is the angle between two vectors."""
        theta_waypoint = math.atan2(waypoint.pose.pose.position.y - y, waypoint.pose.pose.position.x - x)  # Angle of the waypoint relative to the car
        dot_product = np.dot(np.array([math.cos(yaw), math.sin(yaw)]), np.array([math.cos(theta_waypoint), math.sin(theta_waypoint)]))  # Calculate the dot product between two unit vectors, which is equal to |A|*|B|*cos(theta)
        return math.acos(dot_product) < math.pi/2.0

    def waypoints_cb(self, waypoints):
        """Saves the waypoints topic message without the top header."""
        self.base_waypoints = waypoints.waypoints
        rospy.loginfo("waypoint_updater base waypoints obtained")
        pass

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
