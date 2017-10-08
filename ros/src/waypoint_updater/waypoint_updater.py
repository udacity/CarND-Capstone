#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
from waypoint_helper import is_ahead
from waypoint_helper import get_distance_from_waypoint
from waypoint_helper import compose_lane_message
from waypoint_helper import print_waypoint


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

VERBOSE = True       # Print debug logs
LOOKAHEAD_WPS = 200  # Number of waypoints we will publish in /final_waypoints.


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_callback)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.frame_id       = None
        self.current_pose   = None  # current vehicle 3D pose (position + orientation)

        self.base_waypoints = None  # list of all waypoints of the track

        self.loop()

    def loop(self):

        # Set the desired processing rate to 10Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # If there is no waypoint or pose data, wait for some to come in
            if self.base_waypoints is None or self.current_pose is None:
                # Sleep if necessary to maintain the desired processing rate
                rate.sleep()
                continue

            # Find all of the waypoints ahead of the car
            waypoints_ahead = []
            for waypoint in self.base_waypoints:

                if is_ahead(waypoint, self.current_pose):

                    wp_distance = get_distance_from_waypoint(waypoint, self.current_pose)

                    waypoints_ahead.append([waypoint, wp_distance])

            # Sort by distance s.t. the first one is the closest
            waypoints_ahead = sorted(waypoints_ahead, key=lambda x: x[1])

            # Keep only the closest waypoints (also discard distances used to order waypoints)
            waypoints_ahead = [item[0] for item in waypoints_ahead[:LOOKAHEAD_WPS]]

            # Create Lane message with list of waypoints ahead
            lane_message = compose_lane_message(self.frame_id, waypoints_ahead)

            # Publish to final waypoints
            self.final_waypoints_pub.publish(lane_message)

            if VERBOSE:
                print_waypoint(waypoints_ahead[0], msg='Next waypoint: ')

            # Sleep if necessary to maintain the desired processing rate
            rate.sleep()

    def pose_callback(self, msg):
        """
        Update current frame id and vehicle 3D pose (position + orientation)
        """
        self.frame_id = msg.header.frame_id
        self.current_pose = msg.pose

    def waypoints_callback(self, waypoints):
        """
        Store the list of all waypoints.
        Notice that publisher for `/base_waypoints` publishes only once.
        """
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement it later
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. Implement it later
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
