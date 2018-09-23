#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32

import numpy as np
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this
# number
MAX_DECEL = 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # for simulator only, get the traffic lights data
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        # below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
#         rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size = 1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.heading = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.traffic_light = None

        # rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                #closest_waypoint_idx = self.get_closest_waypoint_idx()
                #self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self, x, y):
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

#     def publish_waypoints(self, closest_idx):
#         lane = Lane()
#         lane.header = self.base_waypoints.header
#         lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
#         self.final_waypoints_pub.publish(lane)
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx(self.pose.pose.position.x, self.pose.pose.position.y)
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        rospy.loginfo("decelerate_waypoints")
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            # dist = max(0.0, self.distance(waypoints, i, stop_idx))
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # vel =0.5 * (2 * MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        if self.pose is not None:
            dx = msg.pose.position.x - self.pose.pose.position.x
            dy = msg.pose.position.y - self.pose.pose.position.y
            self.heading = np.arctan2(dx, dy)
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        pass

    def traffic_lights_cb(self, msg):
        # filter by distance
        self.stopline_wp_idx = -1

        if self.heading:
            for light in msg.lights:
                dx = light.pose.pose.position.x - self.pose.pose.position.x
                dy = light.pose.pose.position.y - self.pose.pose.position.y
                dd = (dx**2 + dy**2)**0.5
                dheading = np.arctan2(dx, dy) - self.heading
                while(dheading < 0): dheading += np.pi
                while(dheading > np.pi): dheading -= np.pi

                if dd < 100: # 100 m is when the light can be seen in the distance
                    rospy.loginfo("Light Close {} {} {}".format(dd, dheading, light.state))
                    if dheading < (np.pi/2):
                        rospy.loginfo('Light Ahead')
                        idx = self.get_closest_waypoint_idx(light.pose.pose.position.x, light.pose.pose.position.y)
                        if light.state != 2:
                            rospy.loginfo("light red stopping")
                            self.stopline_wp_idx = idx - 10
                        return # only deal with the first found traffic light

        # aim to stop at 30m from the lights


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
#         self.stopline_wp_idx = msg.data
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
