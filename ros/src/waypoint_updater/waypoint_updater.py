#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,
                                                   queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # self.cur_position = msg.pose.position
        # self.cur_orientation = msg.pose.orientation
        # self.cur_pose = msg.pose
        # did this to understand how to get data out
        # self.cur_x = self.cur_position.x
        # self.cur_y = self.cur_position.y
        # self.cur_z = self.cur_position.z

        # better to use rostopic echo /current_pose
        # rospy.loginfo("current pose (%s, %s, %s) orientation (%s, %s, %s, %s)",
        #               self.cur_x, self.cur_y, self.cur_z,
        #               self.cur_orientation.x, self.cur_orientation.y,
        #               self.cur_orientation.z, self.cur_orientation.w)

        next_wp = self.next_waypoint(self.waypoints, msg.pose)

        rospy.loginfo("current pose (%s, %s) next waypoint (%s, %s)",
                      msg.pose.position.x,
                      msg.pose.position.y,
                      self.waypoints[next_wp].pose.pose.position.x,
                      self.waypoints[next_wp].pose.pose.position.y)

        final_waypoints = Lane()

        for i in range(next_wp, next_wp+LOOKAHEAD_WPS):
            final_waypoints.waypoints.append(self.waypoints[i])

        self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.waypoints = msg.waypoints
        rospy.loginfo("waypoints %s", len(self.waypoints))

    def closest_waypoint(self, waypoints, pose):
        def dl(a, b):
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

        closest_wp = len(waypoints) + 1
        closest_dist = 100000
        for i in range(len(waypoints)):
            dist = dl(waypoints[i].pose.pose.position, pose.position)
            if (dist < closest_dist):
                closest_wp = i
                closest_dist = dist

        return closest_wp

    def next_waypoint(self, waypoints, pose):
        closest_wp = self.closest_waypoint(waypoints, pose)

        waypoint_x = waypoints[closest_wp].pose.pose.position.x
        waypoint_y = waypoints[closest_wp].pose.pose.position.y

        x = pose.position.x
        y = pose.position.y
        heading = math.atan2(waypoint_y - y, waypoint_x - x)

        def quat_array(o):
            return np.array([o.x, o.y, o.z, o.w])

        (roll, pitch, yaw) = euler_from_quaternion(quat_array(pose.orientation))

        angle = abs(yaw-heading)

        if (angle > math.pi / 4):
            closest_wp += 1

        return closest_wp

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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
