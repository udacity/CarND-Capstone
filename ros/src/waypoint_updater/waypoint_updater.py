#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import std_msgs.msg
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
#import tf
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.previous_initial_wp_index = None
        self.traffic = None
        self.current_velocity = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(5) # 5Hz
        while not rospy.is_shutdown():
            if self.pose is not None and self.waypoints is not None and self.traffic is not None and self.current_velocity is not None:
                ego_theta = 2.*math.acos(self.pose.orientation.w)
                ego_vx = np.dot(np.array([[math.cos(ego_theta),-math.sin(ego_theta)],
                                          [math.sin(ego_theta), math.cos(ego_theta)]]), np.array([1,0]))
                ego_vy = np.dot(np.array([[0,-1],[1,0]]), ego_vx)
                #rospy.logerr(ego_theta)

                # publish final_waypoints
                min_distance = 1e9
                initial_wp_index = 0
                for k in xrange(len(self.waypoints)):
                    i = k
                    if self.previous_initial_wp_index is not None:
                        i = (i + self.previous_initial_wp_index) % len(self.waypoints)

                    waypoint = self.waypoints[i].pose.pose.position
                    np_waypoint = np.array([waypoint.x,waypoint.y])
                    np_ego_pose = np.array([self.pose.position.x,self.pose.position.y])
                    np_diff = np_waypoint - np_ego_pose
                    distance = np.linalg.norm(np_diff)
                    param = np.dot(np.linalg.inv(np.vstack((ego_vx,ego_vy)).transpose()), np_diff)
                    # If the waypoint is in front of vehicle and also the closest one,
                    # update the index.
                    if(param[0] > 0 and distance < min_distance):
                        min_distance = distance
                        initial_wp_index = i % len(self.waypoints)
                        if self.previous_initial_wp_index is not None:
                            break

                # publish
                final_waypoints = Lane()
                final_waypoints.header = std_msgs.msg.Header()
                final_waypoints.header.stamp = rospy.Time.now()
                for i in xrange(LOOKAHEAD_WPS):
                    index = (initial_wp_index + i) % len(self.waypoints)
                    final_waypoints.waypoints.append(self.waypoints[index])

                self.previous_initial_wp_index = initial_wp_index

                v_limit = rospy.get_param('/waypoint_loader/velocity') / 3.6  # Speed limit given by ROS parameter
                v_limit *= 0.9           # Set margin to not exceed speed limit.
                v0 = min(20./2.24, v_limit)  # This program allows maximum spped of 20mph.

                if self.traffic == -1:
                    for i in xrange(LOOKAHEAD_WPS):
                        self.set_waypoint_velocity(final_waypoints.waypoints, i, v0)
                else:
                    #                    t0
                    #   v0 ----------------
                    #                      \
                    #                       \
                    #                        \ a0
                    #                         \
                    #                          \
                    #                           \___________ v=0
                    #                           t1
                    #     target velocity diagram
                    #
                    a0 = 2.5        # m/s^2  target acceleration
                    margin = 10      # m      target margin before stop line
                    r0 = self.distance(self.waypoints,initial_wp_index,self.traffic) - margin  # target position to stop
                    t1 = 0.5*(2.*r0/v0 + v0/a0)
                    t0 = 0.5*(2.*r0/v0 - v0/a0)

                    for i in xrange(LOOKAHEAD_WPS):
                        r = self.distance(self.waypoints,initial_wp_index,initial_wp_index+i)
                        if r <= v0 * t0:
                            v = v0
                        elif v0*t0 < r and r < r0:
                            v = math.sqrt(2.*a0*v0*t0 + v0*v0 - 2.*a0*r)
                        else:
                            v = -1
                        self.set_waypoint_velocity(final_waypoints.waypoints, i, v)

                self.final_waypoints_pub.publish(final_waypoints)

            rate.sleep()

    def current_velocity_cb(self,msg):
        self.current_velocity = msg.twist

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        pass

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.waypoints = msg.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic = msg.data
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
