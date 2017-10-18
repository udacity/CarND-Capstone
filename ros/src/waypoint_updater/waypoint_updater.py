#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
import math
import copy
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
        print('WAYPOINT UPDATER111111111111111111111111111111111111111111')

        rospy.init_node('waypoint_updater')

        # Used to determine which waypoints lie ahead of the car
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_callback)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        # Publish the final waypoints for the pure_pursuit node
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Publish waypoints for debugging
        self.pub_car_path = rospy.Publisher('/car_path', Marker, queue_size=1)
        self.pub_next_wp = rospy.Publisher('/next_wp', PoseStamped, queue_size=1)
        self.pub_next_tl = rospy.Publisher('/next_tl', PoseStamped, queue_size=1)

        self.frame_id       = None
        self.current_pose   = None  # current vehicle 3D pose (position + orientation)

        self.base_waypoints = None  # list of all waypoints of the track
        self.num_base_waypoints = 0 # number of all waypoints of the track

        self.tl_index       = -1    # Index of the next traffic light from /traffic_waypoint

        self.loop()

    def loop(self):

        # Set the desired processing rate to 10Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print('Waypoint updater')
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

            # Apply deceleration as necessary
            # TODO: [brahm] Check for traffic light and its state
            if self.tl_index > -1:
                self.apply_deceleration(waypoints_ahead)

            # Create Lane message with list of waypoints ahead
            lane_message = compose_lane_message(self.frame_id, waypoints_ahead)

            # Publish to final waypoints
            self.final_waypoints_pub.publish(lane_message)

            # DEBUGGING
            self.publish_car_path(waypoints_ahead)
            self.publish_next_wp(waypoints_ahead[0])

            if VERBOSE:
                if len(waypoints_ahead) > 0:
                    print_waypoint(waypoints_ahead[0], msg='Next waypoint: ')
                else:
                    print('No waypoints_ahead!')

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
        print('waypoint_updater: p_tl: BASE WAYPOINTS UPDATES')
        self.base_waypoints = waypoints.waypoints
        self.num_base_waypoints = len(self.base_waypoints)

    def traffic_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implement it later

        if VERBOSE:
            print ("traffic_cb: started.")

        # If we are not at the current traffic light waypoint index
        if self.tl_index != msg.data:

            # Update the traffic light index
            self.tl_index = msg.data

            if VERBOSE:
                print ("traffic_cb: changing tl_index: {}".format(self.tl_index))

            # Publish the actual traffic light waypoint for debugging purposes
            if self.base_waypoints is not None and self.tl_index > -1:
                self.publish_next_tl_wp(self.base_waypoints[self.tl_index])

    def apply_deceleration(self, waypoints):
        # TODO: [brahm] implement deceleration to waypoints
        pass

    def publish_car_path(self, waypoints):
        path = Marker()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        path.ns = "path"
        path.id = 0
        path.action = Marker.ADD
        path.type = Marker.LINE_LIST
        path.scale.x = 0.1
        path.color.r = 1.0
        path.color.a = 0.5
        path.points = []
        for waypoint in waypoints:
            position = copy.deepcopy(waypoint.pose.pose.position)
            path.points.append(position)
            position = copy.deepcopy(position)
            position.z = 0.1 # height
            path.points.append(position)
        self.pub_car_path.publish(path)

    def publish_next_wp(self, waypoint):
        next_wp = copy.deepcopy(waypoint.pose)
        next_wp.header.frame_id = self.frame_id
        next_wp.header.stamp = rospy.Time.now()
        self.pub_next_wp.publish(next_wp)

    def publish_next_tl_wp(self, waypoint):
        next_tl = copy.deepcopy(waypoint.pose)
        next_tl.header.frame_id = self.frame_id
        next_tl.header.stamp = rospy.Time.now()
        self.pub_next_tl.publish(next_tl)

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
        n = len(waypoints)
        for i in range(wp1, wp2):
            dist += dl(waypoints[i%n].pose.pose.position, waypoints[(i+1)%n].pose.pose.position)
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
