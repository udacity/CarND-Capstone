#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32


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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
KPH_TO_MPS = 1.0/3.6
DELTA_WP = 40

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # TODO: Add a subscriber for /obstacle_waypoint below

        self.current_velocity = None
        # get current velocity too
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)


        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.num_waypoints = -1

        self.pose = None
        self.pose_stamp = None
        self.prev_position_index = None

        self.car_x = None
        self.car_y = None

        self.car_theta = 0
        self.next_wp_index  = 0

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity')

        # Flags
        self.flag_waypoints_loaded = False
        self.STOPPING_DISTANCE = 25
        self.traffic_waypoint_index = -1
        rospy.logout("self.STOPPING_DISTANCE = %f"%(self.STOPPING_DISTANCE))
        self.loop()
        rospy.spin()

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():

            if self.flag_waypoints_loaded and self.pose is not None:

                self.car_x = self.pose.position.x
                self.car_y = self.pose.position.y

                # Now get the next waypoint....
                self.next_wp_index = self.find_closest_waypoint()

                msg = Lane()
                msg.waypoints = []

                #start_index is at which index out of 10902 is our car now
                start_index = self.next_wp_index

                current_velocity = self.current_velocity.linear.x if self.current_velocity is not None else 0.0
                road_inex = start_index

                for i in range(LOOKAHEAD_WPS):
                    # index of the trailing waypoints
                    rospy.loginfo("i= %s",i)
                    wp = Waypoint()
                    wp.pose.pose.position.x = self.base_waypoints[road_inex].pose.pose.position.x
                    wp.pose.pose.position.y = self.base_waypoints[road_inex].pose.pose.position.y



                    #here you can intruduce manually the detected red light to overide the input of the traffic light detector
                    #self.traffic_waypoint_index=753  #292 753 2047 2580 6294 7008 8540 9733

                    if (self.traffic_waypoint_index < len(self.base_waypoints) and( self.traffic_waypoint_index > start_index)):
                        # We have red head of front

                        #here the input waypoints have been swaped to make it work properly
                        thisDistance = self.distance(self.base_waypoints,road_inex, self.traffic_waypoint_index)


                        rospy.loginfo("thisDistance= %s",thisDistance)
                        rospy.loginfo("road_inex= %s",road_inex)
                        rospy.loginfo("traffic= %s",self.traffic_waypoint_index)
                        #if we are far full speed
                        if (thisDistance > self.STOPPING_DISTANCE):
                            wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS
                            rospy.loginfo("if1")

                        #if we are in a distance need to stop we brake gradually
                        elif (thisDistance <= self.STOPPING_DISTANCE) and (thisDistance > 3):
                            wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS * (float(thisDistance) / float(self.STOPPING_DISTANCE))

                        #if we are very close full stop
                        elif (thisDistance <= 3) and (thisDistance > 0):
                            wp.twist.twist.linear.x = 0


                    else:
                        wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS

                    msg.waypoints.append(wp)
                    road_inex = (road_inex + 1) % self.num_waypoints

                self.final_waypoints_pub.publish(msg)

            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose
        # get the time stamp. might be useful to calculate latency
        self.pose_stamp = msg.header.stamp

    def waypoints_cb(self, waypoints):
        if not self.flag_waypoints_loaded:
            self.base_waypoints = waypoints.waypoints
            self.num_waypoints = len(self.base_waypoints)
            self.flag_waypoints_loaded = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_index = msg.data
        rospy.logdebug("waypoint_updater:traffic_cb says there is a red light at waypoint %s" , self.traffic_waypoint_index )


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def pnt_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def current_velocity_cb(self, msg):
        ''' Callback for /current_velocity topic
            Simply save the current velocity value
        '''
        self.current_velocity = msg.twist

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.pnt_dist(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def vector_direction(self, ref, a, b):
        # 1. calculate the difference of vectors a/b to the ref vector
        ax = a.x - ref.x
        ay = a.y - ref.y
        az = a.z - ref.z

        bx = b.x - ref.x
        by = b.y - ref.y
        bz = b.z - ref.z

        # 2. take the scalar product of the new vectors
        result = ax * bx + ay * by + az * bz

        # 3. if the result is positive, both point into the same direction
        is_same_dir = result > 0

        return is_same_dir

    def find_closest_waypoint(self):
        pose = self.pose # fix current pose
        min_dist = sys.maxsize # set to large number
        pnt_idx = 0

        start_idx = 0
        search_horizon = self.num_waypoints

        if self.prev_position_index is not None:
            start_idx = (self.prev_position_index - DELTA_WP) % self.num_waypoints
            search_horizon = 2 * DELTA_WP

        for i in range(search_horizon):
            idx = (start_idx + i) % self.num_waypoints
            dist = self.pnt_dist(pose.position, self.base_waypoints[idx].pose.pose.position)
            if dist < min_dist:
                pnt_idx = idx
                min_dist = dist

        self.prev_position_index = pnt_idx

        # check if car has passed the closest waypoint already
        # or if it is the next one
        next_pnt_idx = (pnt_idx + 1) % self.num_waypoints

        # if the closest and the following waypoint point into the same
        # direction then the closest one the one in front
        # otherwise it is behind the vehicle and we use the following
        is_same_dir = self.vector_direction(pose.position,
                self.base_waypoints[pnt_idx].pose.pose.position,
                self.base_waypoints[next_pnt_idx].pose.pose.position)

        if is_same_dir:
            return pnt_idx
        else:
            return next_pnt_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
