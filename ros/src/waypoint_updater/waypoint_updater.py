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

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        # FIXME Only for debugging
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        traffic_light_config_string = rospy.get_param("/traffic_light_config")
        self.traffic_light_config = yaml.load(traffic_light_config_string)
        self.traffic_lights = None
        self.traffic_lights_wps = None

        # FIXME Only for debugging
        self.published_wps = []
        self.iteration = 0

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.wps = None
        self.traffic_wp = None
        self.current_velocity = None
        self.next_wp = None

        self.t = rospy.get_time()

        rospy.spin()
        # self.loop()


    # def loop(self):
    #     rate = rospy.Rate(50) # 50Hz
    #     while not rospy.is_shutdown():
    #         if self.next_wp:
    #             self.publish_final_waypoints(self.next_wp)
    #         rate.sleep()


    def pose_cb(self, msg):
        self.pose = msg.pose
        if self.wps:
            next_wp = self.track.next_waypoint(self.pose.position, self.pose.orientation, self.next_wp)
            # FIXME Only for debugging
            # if self.traffic_lights_wps:
            #     closest_light = 0
            #     for i in range(len(self.traffic_lights_wps)-1):
            #         if self.traffic_lights_wps[i] < next_wp\
            #                 and next_wp <= self.traffic_lights_wps[i+1]:
            #             closest_light = i+1
            #     tl = self.traffic_lights[closest_light]
            #     red = tl.state == TrafficLight.RED
            #     yellow = tl.state == TrafficLight.YELLOW and self.traffic_wp != self.traffic_lights_wps[closest_light]
            #     if red or yellow:
            #         self.traffic_wp = self.traffic_lights_wps[closest_light]
            #     else:
            #         self.traffic_wp = -1
            self.next_wp = next_wp
            rospy.loginfo("Time tt %s", rospy.get_time() - self.t)
            self.t = rospy.get_time()
            self.publish_final_waypoints(next_wp)

    def waypoints_cb(self, msg):
        self.wps = msg.waypoints
        self.track = Track(self.wps)
        self.track.update_traffic_lights(self.traffic_light_config)
        # posn = self.wps[1200].pose.pose.position
        # self.track.plot(1100, 1500, posn)
        # plt.show()
        # exit(0)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def traffic_waypoint_cb(self, msg):
        self.traffic_wp = msg.data

    def traffic_lights_cb(self, msg):
        self.traffic_lights = msg.lights
        if self.wps and not self.traffic_lights_wps:
            self.traffic_lights_wps = []
            for i, [x, y] in enumerate(self.traffic_light_config['stop_line_positions']):
                self.traffic_lights_wps.append(self.track.next_waypoint(
                    Point(x,y,0),
                    self.traffic_lights[i].pose.pose.orientation))
                rospy.loginfo("trafficl %s", self.traffic_lights_wps)

            # for l in self.traffic_lights:
            #     self.traffic_lights_wps.append(self.next_waypoint(l.pose.pose.position, l.pose.pose.orientation))

            # lin = np.array([w.twist.twist.linear.x for w in self.wps])
            # plt.plot(lin[100:])
            # plt.show()

            # dist = [self.distance(self.wps, i, i+1) for i in range(0, len(self.wps) - 1)]
            # plt.plot(dist)
            # plt.show()


    def copy_waypoint(self, wp):
        # w = Waypoint()
        # w.pose.pose.position.x = wp.pose.pose.position.x
        # w.pose.pose.position.y = wp.pose.pose.position.y
        # w.pose.pose.position.z = wp.pose.pose.position.z
        # w.pose.pose.orientation.x = wp.pose.pose.orientation.x
        # w.pose.pose.orientation.y = wp.pose.pose.orientation.y
        # w.pose.pose.orientation.z = wp.pose.pose.orientation.z
        # w.pose.pose.orientation.w = wp.pose.pose.orientation.w
        # w.twist.twist.linear.x = wp.twist.twist.linear.x
        # w.twist.twist.linear.y = wp.twist.twist.linear.y
        # w.twist.twist.linear.z = wp.twist.twist.linear.z
        # w.twist.twist.angular.x = wp.twist.twist.angular.x
        # w.twist.twist.angular.y = wp.twist.twist.angular.y
        # w.twist.twist.angular.z = wp.twist.twist.angular.z
        return wp

    def cruise_trajectory(self, next_wp):
        last_wp = next_wp + LOOKAHEAD_WPS
        wps = [self.copy_waypoint(w) for w in self.wps[next_wp : last_wp]]
        # init_vel = self.current_velocity.linear.x
        # max_inc = 1 * 0.02
        # for i, w in enumerate(wps):
        #     adj_vel = init_vel + max_inc * i
        #     w.twist.twist.linear.x = max(w.twist.twist.linear.x, adj_vel)

        # [self.published_wps.append(w) for w in wps]
        # self.iteration += 1
        # rospy.loginfo("Iteration %s", self.iteration)
        # if (self.iteration > 1000):
        #     xs = [w.pose.pose.position.x for w in self.published_wps]
        #     ys = [w.pose.pose.position.y for w in self.published_wps]
        #     self.track.plot(0, last_wp)
        #     plt.plot(xs, ys, "r+")
        #     plt.show()
        #     exit(0)
        for w in wps: w.twist.twist.linear.x = 10.0
        return wps

    def stop_trajectory(self, next_wp, stop_wp):
        #curr_vel = self.current_velocity.linear.x
        curr_vel = float(self.wps[next_wp].twist.twist.linear.x)
        last_wp = next_wp + LOOKAHEAD_WPS
        wps = [self.copy_waypoint(w) for w in self.wps[next_wp : last_wp]]
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
        return wps

    def publish_final_waypoints(self, next_wp):
        stop_wp = self.traffic_wp
        # stop_wp = self.track.traffic_lights_wps[1]
        # dist = self.distance(self.wps, next_wp, stop_wp)
        # curr_vel = self.current_velocity.linear.x
        # stop_dist = 100 # (curr_vel / MAX_DECEL) * curr_vel * 2
        if stop_wp > -1: # and dist < stop_dist:
            final_wps = self.stop_trajectory(next_wp, stop_wp)
            self.final_waypoints_pub.publish(Lane(None, final_wps))
        else:
            # orient = self.pose.orientation
            # _, _, theta = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            #rospy.loginfo("wp %s x %s y %s o %s", next_wp, self.pose.position.x, self.pose.position.y, theta)
            final_wps = self.cruise_trajectory(next_wp)
            self.final_waypoints_pub.publish(Lane(None, final_wps))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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


class Track(object):

    def __init__(self, wps):
        self.wps = wps
        self.traffic_lights_wps = None

    def next_waypoint(self, position, orient=None, around_wp = None):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        if around_wp:
            cand_wps = self.wps[around_wp: around_wp+20]
        else:
            cand_wps = self.wps
        dist = [dl(w.pose.pose.position, position) for w in cand_wps]
        min_wp = np.argmin(dist)
        if around_wp:
            min_wp += around_wp
        if orient:
            _, _, theta = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            head = lambda a, b: math.atan2(a.y - b.y, a.x - b.x)
            heading = head(self.wps[min_wp].pose.pose.position, position)
            if abs(heading - theta) > np.pi/4:
                min_wp = (min_wp + 1) % len(self.wps)
        return min_wp

    def distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(self.wps[wp1].pose.pose.position, self.wps[i].pose.pose.position)
            wp1 = i
        return dist


    def update_traffic_lights(self, traffic_light_config):
        if not self.traffic_lights_wps:
            self.traffic_lights_wps = []
            for i, [x, y] in enumerate(traffic_light_config['stop_line_positions']):
                self.traffic_lights_wps.append(self.next_waypoint(
                    Point(x, y, 0)))

    def plot(self, start=0, stop=-1, position=None):
        if stop < 0:
            stop = len(self.wps)
        # lights = np.array([[l.pose.pose.position.x, l.pose.pose.position.y] for l in self.traffic_lights])
        # stops = np.array([[self.wps[w].pose.pose.position.x, self.wps[w].pose.pose.position.y] for w in self.traffic_lights_wps])
        wps = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.wps[start:stop]])
        plt.plot(wps[:,0], wps[:,1])
        if position:
            plt.plot(position.x, position.y-100, "r*")
            next_wp = self.next_waypoint(position)
            next_posn = self.wps[next_wp].pose.pose.position
            plt.plot(next_posn.x, next_posn.y, "gs")
        # plt.plot(stops[:,0], stops[:,1], "ro")
        # plt.plot(self.pose.position.x, self.pose.position.y, "g+")
        # plt.show()
        # plt.plot(lights[:,0], lights[:,1], "go")



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
