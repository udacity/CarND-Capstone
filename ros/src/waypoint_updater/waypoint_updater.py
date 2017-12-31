#!/usr/bin/env python

import rospy
import math

import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight

import math
import json

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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
STOP_LINE_THRESHOLD = 30.0  # Number of waypoints between the stop line and the traffic lights
MAX_ACCE = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=8)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=2)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Node-wide variables
        self.base_waypoints = None
        self.total_waypoints = 0
        # TODO: Add other member variables you need below

        # @Bassam
        # A boolean to indicate that there is a red traffic light ahead
        self.red_traffic_light_ahead = True
        # The index of the waypoint of the red traffic light
        self.red_traffic_light_waypoint_idx = -1
        # The index of the closest waypoint to our car
        self.closest_waypoint_idx = -1
        self.current_pos = None
        self.current_velocity = None
        self.max_speed_mps = self.kmph2mps(rospy.get_param('~velocity'))
        rospy.logdebug("Max speed in mps: {}".format(self.max_speed_mps))

        rospy.spin()

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def convert_local(self, waypoint, current_pos):
        # Helper function for converting from global to local coords
        # Grab our waypoint and car position variables
        x_way = waypoint.pose.pose.position.x
        y_way = waypoint.pose.pose.position.y
        x_car = current_pos.pose.position.x
        y_car = current_pos.pose.position.y
        # Convert from Quarternion to Radians
        theta_car = 2 * math.acos(current_pos.pose.orientation.w)
        theta_waypoint = 2 * math.acos(waypoint.pose.pose.orientation.w)

        # Perform coordinate localization
        x_shift = x_way - x_car
        y_shift = y_way - y_car
        x = x_shift * math.cos(0 - theta_car) - y_shift * math.sin(0 - theta_car)
        y = x_shift * math.sin(0 - theta_car) + y_shift * math.cos(0 - theta_car)
        return x, y, theta_car, theta_waypoint

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def pose_cb(self, msg):
        # Callback for our current car position recieved
        self.current_pos = msg
        # x = msg.pose.position.x    #Access x value like this
        # theta = msg.pose.orientation.w # Access w value like this

        if self.base_waypoints is None or self.current_velocity is None:
            return

        # Create list for our published final_waypoints
        final_waypoints_list = []

        # Scan through all base_waypoints
        for i in range(len(self.base_waypoints.waypoints)):
            waypoint = self.base_waypoints.waypoints[i]
            # Convert waypoint to local coordinates
            x, y, theta_car, theta_waypoint = self.convert_local(waypoint, self.current_pos)
            orientation_match = math.cos(theta_waypoint - theta_car)
            # Check if the waypoint is in front of our car, and if the orientation is within +/- pi/4 of our car
            if x >= 0.00 and orientation_match > 0.707:
                self.closest_waypoint_idx = i

                if self.red_traffic_light_ahead:
                    dist_to_stop_line = self.distance(self.base_waypoints.waypoints, self.closest_waypoint_idx,
                                                      self.red_traffic_light_waypoint_idx) - STOP_LINE_THRESHOLD
                    if dist_to_stop_line > 0:
                        # acceleration = (final_v^2 - curr_v^2)/ (2 * distance)
                        DECEL = (0 - self.current_velocity ** 2) / (2 * dist_to_stop_line)
                    else:
                        DECEL = 3.0

                # Since our waypoints are sequential
                # As soon as we find our first waypoint, we populate the rest of the list with the following waypoints
                for j in range(LOOKAHEAD_WPS):
                    j_mod = i + j % self.total_waypoints

                    next_wp = self.base_waypoints.waypoints[j_mod]
                    if self.red_traffic_light_ahead:
                        rospy.logdebug(
                            "Red Traffic light Ahead Idx?: {}  Carla Idx: {} diff: {} distance: {}".format(
                                self.red_traffic_light_waypoint_idx,
                                self.closest_waypoint_idx,
                                self.red_traffic_light_waypoint_idx - self.closest_waypoint_idx,
                                dist_to_stop_line))
                        # wp_count = (
                        #         self.red_traffic_light_waypoint_idx - STOP_LINE_THRESHOLD - self.closest_waypoint_idx)
                        # rospy.logdebug("Red Traffic Light Waypoints diff: {}".format(wp_count))
                        if -(STOP_LINE_THRESHOLD/4) < dist_to_stop_line <= 4 * STOP_LINE_THRESHOLD:
                            dist = self.distance(self.base_waypoints.waypoints, j_mod, j_mod + 2)
                            if self.current_velocity < 0.1:
                                vel = 0.0
                            else:
                                vel = math.sqrt(abs(self.current_velocity ** 2 + (2 * DECEL * dist)))
                                if vel < 1.:
                                    vel = 0.
                            rospy.logdebug(
                                "Current velocity: {} Target velocity: {}".format(self.current_velocity, vel))
                            next_wp.twist.twist.linear.x = max(0, vel)
                            if j:
                                rospy.logdebug(
                                    "Stop Next waypoint idx: {}  velocity: {} distance: {}"
                                        .format(j, next_wp.twist.twist.linear.x, dist))
                            # else:
                            #     next_wp.twist.twist.linear.x = 0
                        else:
                            next_wp.twist.twist.linear.x = min(self.current_velocity + ((j + 1) * 0.5),
                                                               self.max_speed_mps)

                    else:
                        next_wp.twist.twist.linear.x = min((self.current_velocity + (j + 1) * MAX_ACCE), self.max_speed_mps)
                        # rospy.logdebug(
                        #     "Go Next waypoint idx: {}  velocity: {}".format(j, next_wp.twist.twist.linear.x))

                    final_waypoints_list.append(next_wp)
                # Format our message
                msg = Lane()
                msg.waypoints = final_waypoints_list
                # Setting header info to current_pos header
                msg.header = self.current_pos.header
                self.final_waypoints_pub.publish(msg)
                return

    def waypoints_cb(self, waypoints):
        # Call back for base_waypoints when our simulator is started
        # first_x = waypoints.waypoints[0].twist.twist.linear.x #Use this format to access waypoint info

        # Set a variable for accessing base_waypoints throughout this node
        self.base_waypoints = waypoints
        # Set a variable for access the total number of base_waypoints throughout this node
        self.total_waypoints = len(self.base_waypoints.waypoints)
        # rospy.logerr("Number of waypoints: {}".format(self.total_waypoints))

    def get_closest_waypoint_idx(self, start_idx, pose):
        """Find the index of the traffic light starting from start_idx"""
        waypoint_idx = -1
        closest_dist = sys.maxint

        for i in range(self.total_waypoints):
            waypoint_position = self.base_waypoints.waypoints[(i + start_idx) % self.total_waypoints].pose.pose.position
            tl_position = pose.pose.position
            dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
            dist = dl(waypoint_position, tl_position)
            if dist < closest_dist:
                waypoint_idx = i
                closest_dist = dist

        return waypoint_idx

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        if self.current_velocity is None or self.current_pos is None or self.base_waypoints is None:
            return

        # @Bassam: For testing purpose we are subscribing to the /vehicle/traffic_lights
        traffic_lights = msg.lights
        closest_tl = None
        closest_dist = sys.maxint  # Some big number

        for i in range(len(traffic_lights)):
            tl = msg.lights[i]
            if tl.state == TrafficLight.RED or tl.state == TrafficLight.YELLOW:  # Red or Yellow Traffic light
                rospy.logdebug("TL state: {}".format(int(tl.state)))
                # Convert traffic light location to car local coordinates
                x, y, theta_car, theta_tl = self.convert_local(tl, self.current_pos)
                dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
                dist = dl(tl.pose.pose.position,
                          self.base_waypoints.waypoints[self.closest_waypoint_idx].pose.pose.position)
                if x > 0.0 and 0 < dist < closest_dist:
                    closest_tl = tl
                    closest_dist = dist
                # rospy.logdebug("Car theta: {}     tl theta:{}  Dist: {}"
                #                .format((theta_car), (theta_tl), dist))
                # Check if the traffic light is in front of our car, and if the orientation is within +/- pi/4 of our car
                # I have seen other traffic lights that are sideways to the car track
        if closest_tl is not None:
            tl_waypoint_idx = self.get_closest_waypoint_idx(0, closest_tl.pose)
            self.red_traffic_light_waypoint_idx = tl_waypoint_idx
            self.red_traffic_light_ahead = True

            # rospy.logdebug("Closest Red traffic light {} meters ahead".format(dist))
        else:
            self.red_traffic_light_ahead = False

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
