#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, String

import math
import numpy as np
from scipy.spatial import KDTree

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # test publisher for debugging
        # self.test = rospy.Publisher('/test', String, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.car_velocity = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            # if pose (from simulator) and waypoints (from waypoint_loader) are received
            if self.pose and self.waypoint_tree:
                # get list of closest waypoints
                closest_waypoint_idx = self.get_closest_waypoint_id()
                # publish list of closest waypoints ahead of vehicle
                self.publish_waypoints(closest_waypoint_idx)

            rate.sleep()

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # closest waypoint ahead or behind vehicle?
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane(closest_idx)
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self, closest_idx):
        lane = Lane()

        farthest_idx = closest_idx + LOOKAHEAD_WPS
        car_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # hard coded to only run at target speed to verify DBW_node and PID loop
        self.stopline_wp_idx = -1

        # no red light in sight
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = car_waypoints
        # red light coming up (in max waypoint range)
        else:
            lane.waypoints = self.decelerate_waypoints(car_waypoints, closest_idx)

        return lane


    def decelerate_waypoints(self, waypoints, closest_idx):
        decel_waypoints = []
        # create stop index for center of the car relative to the stop line
        stop_idx = max(self.stopline_wp_idx - closest_idx -4, 0)
        # calculate distance from car to stopping point
        car_stop_distance = self.distance(waypoints, 0, stop_idx)

        # calculate linear deceleration value: a = (v)2 / (2 * s)
        if car_stop_distance <= 0.2:
        	needed_decel = MAX_DECEL
        else:
        	needed_decel = (self.car_velocity * self.car_velocity) / (2 * car_stop_distance)

        for i, wp in enumerate(waypoints):
            # create a waypoint and copy the original x, y, z position data
            p = Waypoint()
            p.pose = wp.pose
            
            # keep the speed for current waypoint of the car
            if (i == 0):
                vel = self.car_velocity
            # reduce speed for all following waypoints
            else:
                dist = self.distance(waypoints, i, stop_idx)

                # set velocity to zero for all waypoints behind red light
                if (dist <= 0.):
                    vel = 0.
                else:
                    delta_s = car_stop_distance - dist
                    delta_v = math.sqrt(2*delta_s*needed_decel)

                    vel = self.car_velocity - delta_v

            p.twist.twist.linear.x = vel
            decel_waypoints.append(p)

        return decel_waypoints



    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def velocity_cb(self, msg):
        self.car_velocity = msg.twist.linear.x        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

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
