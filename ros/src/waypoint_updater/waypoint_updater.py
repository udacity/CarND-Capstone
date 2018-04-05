#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
import tf
import math
from copy import deepcopy

'''
This node will publish waypoints from the car's current position to some `x`
distance ahead.

As mentioned in the doc, you should ideally first implement a version which
does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of
traffic lights too.

Please note that our simulator also provides the exact location of traffic
lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node as well as to verify your TL
classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber(
            '/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)
        self.base_waypoints = []
        self.next_wp_idx = -1
        self.next_stop_line_idx = -1
          
        rospy.spin()

    def closest_waypoint(self, curr_pose):
        closest_distance = float('inf')
        closest_idx = -1
        for idx, waypoint in enumerate(self.base_waypoints):
            distance = self.straight_dist(
                waypoint.pose.pose.position,
                curr_pose.pose.position)
            if distance < closest_distance:
                closest_idx = idx
                closest_distance = distance
        curr_yaw = self.compute_yaw(curr_pose.pose.orientation)

        map_x = self.base_waypoints[closest_idx].pose.pose.position.x
        map_y = self.base_waypoints[closest_idx].pose.pose.position.y
        heading = math.atan2(
            (map_y - curr_pose.pose.position.y),
            (map_x - curr_pose.pose.position.x)
        )

        if abs(curr_yaw - heading) > math.pi / 4:
            return closest_idx + 1
        else:
            return closest_idx

    def compute_yaw(self, orientation):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
        return yaw

    def pose_cb(self, pose_stamped):
        self.next_wp_idx = self.closest_waypoint(pose_stamped)

        waypoints = self.base_waypoints[
            self.next_wp_idx:self.next_wp_idx+LOOKAHEAD_WPS

        ]
        if self.next_stop_line_idx != -1:
            waypoints = self.slowdown_to_stop(waypoints)

        final_waypoints_msg = Lane()
        final_waypoints_msg.waypoints = waypoints

        self.final_waypoints_pub.publish(final_waypoints_msg)

    def slowdown_to_stop(self, waypoints):
        last_idx = self.next_stop_line_idx-self.next_wp_idx
        last = waypoints[last_idx]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:last_idx][::-1]:
            dist = self.straight_dist(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        stop_waypoint_idx = msg.data
        if stop_waypoint_idx == -1:
            self.next_stop_line_idx = -1
        else:
            self.next_stop_line_idx = stop_waypoint_idx

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def straight_dist(self, pos0, pos1):
        return math.sqrt((pos0.x - pos1.x) ** 2 + (pos0.y - pos1.y) ** 2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.straight_dist(
                waypoints[wp1].pose.pose.position,
                waypoints[i].pose.pose.position
            )
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
