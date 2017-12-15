#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy
import tf.transformations   # to get Euler coordinates
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

LOOKAHEAD_WPS = 50      # Number of waypoints we will publish. You can change this number

# Profile for slowing at traffic light: v = K_SLOW * sqrt(dist - DIST_MIN)
# This is equivalent to considering a constant deceleration
K_SLOW = 5     # in m^1/2 . s^-1
DIST_MIN = 10   # distance we need to be from stop line


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.ego_pos = None
        self.wps = None
        self.final_wps = None
        self.first_pass = True
        self.red_light_wp_idx = -1 # -1 is representing traffic light is not red
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        # Logging data in csv file
        self.log_to_csv = True
        if self.log_to_csv:
            self.log_handle = self.log_init('waypoint_updater.csv')

        self.loop()

    def loop(self):

        rate = rospy.Rate(5)

        while not all([self.wps, self.ego_pos]):

            rate.sleep()

        while not rospy.is_shutdown() and all([self.wps, self.ego_pos, self.final_wps]):

            # Get car orientation
            car_x, car_y = self.ego_pos.position.x, self.ego_pos.position.y
            quaternion = (self.ego_pos.orientation.x, self.ego_pos.orientation.y,
                          self.ego_pos.orientation.z, self.ego_pos.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            car_yaw = euler[2]

            #return the index of the closest waypoint ahead of us
            closest_idx_waypoint = self.closest_waypoint_ahead(car_x, car_y, car_yaw, self.wps.waypoints)

            #final waypoints is a subset of original set of waypoints
            self.final_wps.waypoints = self.wps.waypoints[closest_idx_waypoint:closest_idx_waypoint+LOOKAHEAD_WPS]

            #check we didn't reach the end of the list and otherwise loopback to start of the list
            if len(self.final_wps.waypoints) < LOOKAHEAD_WPS:
                extra_points_needed = LOOKAHEAD_WPS - len(self.final_wps.waypoints)

                # we need to get points from the start of the list ensuring next point is closest ahead
                last_x = self.wps.waypoints[-1].pose.pose.position.x
                last_y = self.wps.waypoints[-1].pose.pose.position.y
                last_x2 = self.wps.waypoints[-2].pose.pose.position.x
                last_y2 = self.wps.waypoints[-2].pose.pose.position.y
                last_yaw = math.atan2(last_y - last_y2, last_x - last_x2)
                # we don't include last points of the list to ensure we go back to the beginning of the list
                first_extra_point = self.closest_waypoint_ahead(last_x, last_y, last_yaw, self.wps.waypoints[0:-10])

                # we complete our list to desired number of points
                self.final_wps.waypoints.extend(self.wps.waypoints[first_extra_point:first_extra_point+extra_points_needed])

            if len(self.final_wps.waypoints) != LOOKAHEAD_WPS:
                rospy.logwarn("List of /final_waypoints does not contain target number of elements")

            # if traffic light is red
            if self.red_light_wp_idx != -1:

                # we make a copy of the data to ensure it does not modify referenced base points
                self.final_wps = copy.deepcopy(self.final_wps)

                # find index of waypoint corresponding to traffic light in final_wps
                red_idx_final_wps = (self.red_light_wp_idx - closest_idx_waypoint) % len(self.wps.waypoints)

                # get position of traffic light
                traffic_light_waypoint = self.wps.waypoints[self.red_light_wp_idx]

                # if it is far, we start reducing the speed slowly until our horizon
                if red_idx_final_wps >= LOOKAHEAD_WPS:
                    red_idx_final_wps = LOOKAHEAD_WPS - 1

                # Reduce velocity based on distance to light
                for idx in range(red_idx_final_wps + 1):
                    distance_to_light = math.sqrt(self.distance_sq_between_waypoints(self.final_wps.waypoints[idx], traffic_light_waypoint))
                    # use a profile based on constant deceleration
                    distance_to_stop = max(distance_to_light - DIST_MIN, 0)
                    self.final_wps.waypoints[idx].twist.twist.linear.x = min(K_SLOW * math.sqrt(distance_to_stop) * 1000 / 3600,
                                                                            self.final_wps.waypoints[idx].twist.twist.linear.x)

                # Set all future points to zero speed
                for idx in range(red_idx_final_wps + 1, len(self.final_wps.waypoints) - 1):
                    self.final_wps.waypoints[idx].twist.twist.linear.x = 0                

            self.final_waypoints_pub.publish(self.final_wps)
            
            if self.log_to_csv:
                    self.log_data(rospy.get_rostime(), car_x, car_y, closest_idx_waypoint, self.wps.waypoints[closest_idx_waypoint].pose.pose.position.x, self.wps.waypoints[closest_idx_waypoint].pose.pose.position.y)
 
            
            rate.sleep()

    def pose_cb(self, msg):
        self.ego_pos = msg.pose

    def waypoints_cb(self, waypoints):
        # Ensure we only get initial full list of waypoints as simulator keeps publishing
        # with patial list aftewards
        if self.wps is None:
            # We need to get a full copy as otherwise we just get a reference
            self.wps = copy.copy(waypoints)
            self.final_wps = copy.copy(waypoints)

    def traffic_cb(self, msg):
        # red_light_wp_idx is representing red traffic light waypoint ahead of current vehicle position.
        self.red_light_wp_idx = msg.data

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
        for i in range(wp1+1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_sq_between_waypoints(self, wp1, wp2):
        dl = lambda a, b: (a.x - b.x) ** 2 + (a.y-b.y)**2  + (a.z-b.z)**2
        return dl(wp1.pose.pose.position, wp2.pose.pose.position)

    def closest_waypoint_ahead(self, pos_x, pos_y, yaw, waypoints):
        ''' Return index of closest point ahead '''

        # Create some logging info
        loginfo = 'yaw: {} | x: {} | y: {}'.format(yaw, pos_x, pos_y)

        # Define unit vector for car orientation in global (x, y) coordinates
        orient_x, orient_y = math.cos(yaw), math.sin(yaw)

        # Filter waypoints to keep only the ones ahead of us by checking scalar product
        waypoints_ahead = [(n, wp) for (n, wp) in enumerate(waypoints)
                           if (orient_x * (wp.pose.pose.position.x - pos_x) +
                           orient_y * (wp.pose.pose.position.y - pos_y)) > 0]
        if not len(waypoints_ahead):
            rospy.logwarn("No points detected ahead of us")
        
        # Extract closest waypoint
        closest_waypoint = min(waypoints_ahead,
                               key = lambda wpidx: (wpidx[1].pose.pose.position.x - self.ego_pos.position.x) ** 2
                               + (wpidx[1].pose.pose.position.y - self.ego_pos.position.y) ** 2)
        closest_index = closest_waypoint[0]
        loginfo += '| Closest waypoint index: {}'.format(closest_index)
        rospy.logdebug_throttle(1, loginfo)

        return closest_index

    def log_init(self, log_path):
        log_handle = open(log_path,'w')
        return log_handle

    def log_data(self, *args):
        self.log_handle.write(','.join(str(arg) for arg in args) + '\n')


if __name__ == '__main__':
    try:
	WaypointUpdater()	
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
