#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=2)

        self.waypoints = None
        self.latest_pose = None
        self.num_waypoints = 0
        self.closest_waypoint = 0
        self.next_red_light = None
        self.MAX_VELOCITY = rospy.get_param("~max_velocity")
        self.REDUCE_SPEED_DISTANCE = rospy.get_param("~reduce_speed_distance")
        self.STOP_DISTANCE = rospy.get_param("~stop_distance")

        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.calculate_and_publish_next_waypoints()
            r.sleep()

    def calculate_waypoint_velocity(self, waypoint_index):
        velocity = self.MAX_VELOCITY

        if self.next_red_light and self.waypoints:
            distance_to_red_light = self.distance(self.waypoints, waypoint_index, self.next_red_light)
            if (distance_to_red_light < self.STOP_DISTANCE):
                velocity = 0.0
            elif (distance_to_red_light < self.REDUCE_SPEED_DISTANCE):
                ratio = distance_to_red_light / self.REDUCE_SPEED_DISTANCE
                velocity = self.MAX_VELOCITY * ratio

        if velocity > self.MAX_VELOCITY:
            velocity = self.MAX_VELOCITY

        return velocity

    def calculate_and_publish_next_waypoints(self):
        wps = []
        if (self.latest_pose and self.waypoints):
            ix = self.next_waypoint(self.waypoints, self.latest_pose.pose)
            self.closest_waypoint = ix
            ix_end = ix+LOOKAHEAD_WPS
            if ix_end > self.num_waypoints:
                ix_end = self.num_waypoints
            for i in range(ix, ix_end):
                self.set_waypoint_velocity(self.waypoints, i, self.calculate_waypoint_velocity(i))
                wp = self.waypoints[i]
                wps.append(wp)

        final_wps = Lane()
        final_wps.waypoints = wps

        self.final_waypoints_pub.publish(final_wps)

    def pose_cb(self, msg):
        self.latest_pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.num_waypoints = len(self.waypoints)
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        if (msg.data >= 0):
            self.next_red_light = msg.data
        else:
            self.next_red_light = None

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

    def find_closest_waypoint(self, waypoints, pose):

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)#  + (a.z-b.z)**2)

        closest_len = 100000

        # no need to start from 0, instead start looking from closest wp from 'just' previous run
        if self.closest_waypoint > 20:
            closest_waypoint = self.closest_waypoint - 20#0
            next_waypoint = self.closest_waypoint -20 #0
        else:
            closest_waypoint = 0
            next_waypoint = 0

        num_waypoints = self.num_waypoints
        dist = dl(waypoints[closest_waypoint].pose.pose.position, pose.position)

        while (dist < closest_len) and (closest_waypoint < num_waypoints):
            closest_waypoint = next_waypoint
            closest_len = dist
            dist = dl(waypoints[closest_waypoint+1].pose.pose.position, pose.position)
            next_waypoint += 1

        dist_prev = dl(waypoints[closest_waypoint-1].pose.pose.position, pose.position)
        dist_curr = dl(waypoints[closest_waypoint].pose.pose.position, pose.position)
        dist_next = dl(waypoints[closest_waypoint+1].pose.pose.position, pose.position)

        #rospy.loginfo("""Waypoint dist {} {} {}""".format(dist_prev, dist_curr, dist_next))

        return closest_waypoint

    def next_waypoint(self, waypoints, pose):

        closest_waypoint = self.find_closest_waypoint(waypoints, pose)

        pose_x = pose.position.x
        pose_y = pose.position.y

        pose_orient_x = pose.orientation.x
        pose_orient_y = pose.orientation.y
        pose_orient_z = pose.orientation.z
        pose_orient_w = pose.orientation.w

        quaternion = (pose_orient_x, pose_orient_y, pose_orient_z, pose_orient_w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        pose_yaw = euler[2]

        wp = waypoints[closest_waypoint]
        wp_x = wp.pose.pose.position.x
        wp_y = wp.pose.pose.position.y

        heading = math.atan2((wp_y - pose_y),(wp_x - pose_x))

        if (pose_yaw > (math.pi/4)):
            closest_waypoint += 1

        return closest_waypoint



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
