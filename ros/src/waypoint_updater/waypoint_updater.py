#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32

import math
import numpy as np
import tf
import yaml
import matplotlib.pyplot as plt
import copy

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

MAX_DECEL = 5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.wps = None
        self.wps_base = None
        self.traffic_wp = -1
        self.next_wp = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        if self.wps and self.wps_base:
            next_wp = self.next_waypoint(self.pose.position, self.pose.orientation, self.next_wp)
            self.next_wp = next_wp
            self.publish_final_waypoints(next_wp)
        # self.plot(stop=1000, pose=self.pose)
        # plt.show()
        # exit(0)


    def waypoints_cb(self, msg):
        self.wps = msg.waypoints
        self.wps_base = copy.deepcopy(self.wps)
        for w in self.wps: w.twist.twist.linear.x = 0

    def cruise_trajectory(self, next_wp):
        curr_vel = float(self.wps[next_wp-1].twist.twist.linear.x)
        last_wp = next_wp + LOOKAHEAD_WPS
        wps = self.wps[next_wp : last_wp]
        wps_base = self.wps_base[next_wp : last_wp]
        # inc_vel = (self.wps_base[next_wp-1].twist.twist.linear.x - curr_vel)/float(last_wp - next_wp)
        inc_vel = 1.0
        for i, w in enumerate(wps):
            w.twist.twist.linear.x = curr_vel + inc_vel * (i+1)
            w.twist.twist.linear.x = min(w.twist.twist.linear.x, wps_base[i].twist.twist.linear.x)
        # sample = [wps[i].twist.twist.linear.x for i in range(0, 20, 2)]
        # rospy.loginfo("wp init cruise %s %s", inc_vel, sample)
        return wps

    def stop_trajectory(self, next_wp, stop_wp):
        curr_vel = float(self.wps[next_wp].twist.twist.linear.x)
        last_wp = next_wp + LOOKAHEAD_WPS
        wps = self.wps[next_wp : last_wp]
        wps_base = self.wps_base[next_wp : last_wp]
        gap = stop_wp - next_wp
        if gap > 0:
            dec_vel = curr_vel/gap
            init_vel = curr_vel
        else:
            dec_vel = 0
            init_vel = 0
        for i, w in enumerate(wps):
            w.twist.twist.linear.x = init_vel - dec_vel * i
            if w.twist.twist.linear.x < 0.5: w.twist.twist.linear.x = 0
            w.twist.twist.linear.x = min(w.twist.twist.linear.x, wps_base[i].twist.twist.linear.x)
        sample = [wps[i].twist.twist.linear.x for i in range(0, 20, 2)]
        rospy.loginfo("wp init stop %s ", sample)
        return wps

    def publish_final_waypoints(self, next_wp):
        stop_wp = self.traffic_wp
        if stop_wp > -1:
            dist = self.distance(self.wps, next_wp, stop_wp)
            stop_dist = 100  # (curr_vel / MAX_DECEL) * curr_vel * 2
            if dist < stop_dist:
                final_wps = self.stop_trajectory(next_wp, stop_wp)
                self.final_waypoints_pub.publish(Lane(None, final_wps))
                return
        final_wps = self.cruise_trajectory(next_wp)
        self.final_waypoints_pub.publish(Lane(None, final_wps))

    def traffic_cb(self, msg):
        self.traffic_wp = msg.data

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

    def next_waypoint(self, position, orient=None, around_wp = None):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        SPAN_WP = 20
        if around_wp:
            cand_wps = [self.wps[i % len(self.wps)] for i in range(around_wp, around_wp + SPAN_WP)]
        else:
            cand_wps = self.wps
        dist = [dl(w.pose.pose.position, position) for w in cand_wps]
        min_wp = np.argmin(dist)
        if around_wp:
            min_wp = (min_wp + around_wp) % len(self.wps)
        if orient:
            _, _, theta = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            head = lambda a, b: math.atan2(a.y - b.y, a.x - b.x)
            heading = head(self.wps[min_wp].pose.pose.position, position)
            if abs(heading - theta) > np.pi/4:
                min_wp = (min_wp + 1) % len(self.wps)
        return min_wp

    def plot(self, start=0, stop=-1, pose=None, plot_wps=None):
        if stop < 0:
            stop = len(self.wps)
        # lights = np.array([[l.pose.pose.position.x, l.pose.pose.position.y] for l in self.traffic_lights])
        # stops = np.array([[self.wps[w].pose.pose.position.x, self.wps[w].pose.pose.position.y] for w in self.traffic_lights_wps])
        wps = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.wps[start:stop]])
        plt.plot(wps[:,0], wps[:,1])
        if pose:
            position, orient = pose.position, pose.orientation
            _, _, theta = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            plt.plot(position.x, position.y, "rs")
            plt.plot(position.x+20*math.cos(theta), position.y+20*math.sin(theta), "r*")
        if plot_wps:
            plt.plot([w.pose.pose.position.x for w in plot_wps], [w.pose.pose.position.y for w in plot_wps], "g.")



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
