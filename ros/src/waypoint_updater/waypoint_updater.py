#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
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
MAX_DECEL = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic_wp_idx = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /obstacle_waypoint

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoints and self.waypoints_2d and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            # closest_coord is behind us
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_wp_idx):
        lane = self.generate_lane(closest_wp_idx)
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self, closest_wp_idx):
        farthest_wp_idx = closest_wp_idx + LOOKAHEAD_WPS
        base_waypoints = self.waypoints.waypoints[closest_wp_idx : farthest_wp_idx]

        lane = Lane()

        if self.traffic_wp_idx is None or \
            (self.traffic_wp_idx < 0) or \
            (self.traffic_wp_idx >= farthest_wp_idx):

            lane.waypoints = base_waypoints
        else:
            # Stop 2 waypoints before closest waypoint to the traffic light
            stop_idx = max(self.traffic_wp_idx - closest_wp_idx - 2, 0)

            lane.waypoints = self.decelerate_waypoints(base_waypoints, stop_idx)

        return lane

    def decelerate_waypoints(self, waypoints, stop_idx):
        waypoints_up_to_stop = waypoints[:stop_idx]
        dists_to_stop = self.distances_to_end(waypoints_up_to_stop)

        result = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            if i >= stop_idx:
                vel = 0
            else:
                dist_to_stop = dists_to_stop[i]
                
                # Linear decrease in velocity wrt time. We could smoothen this.
                vel = math.sqrt(2 * MAX_DECEL * dist_to_stop)

                # obey speed limit
                vel = min(vel, wp.twist.twist.linear.x)
            p.twist.twist.linear.x = vel
            result.append(p)

        return result

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [
                [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                for waypoint in waypoints.waypoints
            ]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        if self.traffic_wp_idx != msg.data:
            rospy.loginfo('waypoint_updater received new traffic wp {}'.format(msg.data))
        self.traffic_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def distances_to_end(self, waypoints):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        len_wps = len(waypoints)

        dists_reversed = [0]

        for i in range(len_wps - 1):
            wp_idx = len_wps - 2 - i

            incremental_dist = dl(
                waypoints[wp_idx].pose.pose.position,
                waypoints[wp_idx + 1].pose.pose.position)

            total_dist = dists_reversed[-1] + incremental_dist

            dists_reversed.append(total_dist)

        return dists_reversed[::-1]


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
