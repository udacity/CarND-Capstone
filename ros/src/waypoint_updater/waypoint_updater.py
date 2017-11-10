#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)


        # FIXME Only for debugging
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        traffic_light_config_string = rospy.get_param("/traffic_light_config")
        self.traffic_light_config = yaml.load(traffic_light_config_string)
        self.traffic_lights = None
        self.traffic_lights_wps = None

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.wps = None
        self.traffic_wp = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        if self.wps:
            next_wp = self.next_waypoint(self.pose)
            # FIXME Only for debugging
            if self.traffic_lights_wps:
                closest_light = 0
                for i in range(len(self.traffic_lights_wps)):
                    if self.traffic_lights_wps[i] < next_wp\
                            and next_wp <= self.traffic_lights_wps[i+1]:
                        closest_light = i+1
                if self.traffic_lights[closest_light].state == TrafficLight.RED:
                    self.traffic_wp = self.traffic_lights_wps[closest_light]
                else:
                    self.traffic_wp = -1
            self.publish_final_waypoints(next_wp)

    def waypoints_cb(self, msg):
        self.wps = msg.waypoints

    def traffic_waypoint_cb(self, msg):
        self.traffic_wp = msg

    def traffic_lights_cb(self, msg):
        self.traffic_lights = msg.lights
        if self.wps and not self.traffic_lights_wps:
            self.traffic_lights_wps = [self.next_waypoint(l.pose.pose) for l in self.traffic_lights]
        # lights = np.array([[l.pose.pose.position.x, l.pose.pose.position.y] for l in self.traffic_lights])
        # stops = np.array(self.traffic_light_config['stop_line_positions'])
        # wps = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.wps])
        # plt.plot(wps[:,0], wps[:,1], "b.")
        # plt.plot(stops[:,0], stops[:,1], "ro")
        # plt.plot(lights[:,0], lights[:,1], "go")
        # plt.show()


    def copy_waypoint(self, wp):
        w = Waypoint()
        w.pose.pose.position.x = wp.pose.pose.position.x
        w.pose.pose.position.y = wp.pose.pose.position.y
        w.pose.pose.position.z = wp.pose.pose.position.z
        w.pose.pose.orientation.x = wp.pose.pose.orientation.x
        w.pose.pose.orientation.y = wp.pose.pose.orientation.y
        w.pose.pose.orientation.z = wp.pose.pose.orientation.z
        w.pose.pose.orientation.w = wp.pose.pose.orientation.w
        w.twist.twist.linear.x = wp.twist.twist.linear.x
        w.twist.twist.linear.y = wp.twist.twist.linear.y
        w.twist.twist.linear.z = wp.twist.twist.linear.z
        w.twist.twist.angular.x = wp.twist.twist.angular.x
        w.twist.twist.angular.y = wp.twist.twist.angular.y
        w.twist.twist.angular.z = wp.twist.twist.angular.z
        return w

    def next_waypoint(self, pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        head = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        o = pose.orientation
        _, _, theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        dist = [dl(w.pose.pose.position, pose.position) for w in self.wps]
        min_wp = np.argmin(dist)
        heading = head(self.wps[min_wp].pose.pose.position, pose.position)
        if abs(heading - theta) > np.pi/4:
            min_wp = (min_wp + 1) % len(self.wps)
        return min_wp

    def publish_final_waypoints(self, next_wp):
        last_wp = next_wp + LOOKAHEAD_WPS
        final_wps = [self.copy_waypoint(w) for w in self.wps[next_wp : last_wp]]
        if self.traffic_wp and next_wp <= self.traffic_wp and self.traffic_wp <= last_wp:
            for w in final_wps:
                w.twist.twist.linear.x = 0
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
