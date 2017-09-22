#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number

MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints',
                                                   Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,
                                                   queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None

        self.prev_pose = None
        self.prev_next_wp = None

        self.redlight_waypoint = None

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

        if (self.waypoints is None):
            rospy.logwarn("Received pose_cb before initial waypoints_cb")
            return

        if (self.prev_pose == msg.pose):
            next_wp = self.prev_next_wp
            rospy.loginfo("same as previous pose (%s, %s) next waypoint (%s, %s) %s",
                          msg.pose.position.x,
                          msg.pose.position.y,
                          self.waypoints[next_wp].pose.pose.position.x,
                          self.waypoints[next_wp].pose.pose.position.y,
                          next_wp)
        else:
            next_wp = self.next_waypoint(self.waypoints, msg.pose)
            rospy.loginfo("current pose (%s, %s) next waypoint (%s, %s) %s",
                          msg.pose.position.x,
                          msg.pose.position.y,
                          self.waypoints[next_wp].pose.pose.position.x,
                          self.waypoints[next_wp].pose.pose.position.y,
                          next_wp)
            self.prev_next_wp = next_wp
            self.prev_pose = msg.pose

        final_waypoints = Lane()

        final_wp = next_wp+LOOKAHEAD_WPS
        if final_wp > len(self.waypoints)-1:
            final_wp = len(self.waypoints)-1
        for i in range(next_wp, final_wp):
            final_waypoints.waypoints.append(self.waypoints[i])

            # only add waypoints up till the traffic light
            if self.redlight_waypoint is not None:
                if i == self.redlight_waypoint:
                    rospy.loginfo("stopped adding final waypoints past redlight %s", i)
                    break

        # only decellerate if this redlight is in the waypoints we have
        if self.redlight_waypoint is not None:
            if next_wp <= self.redlight_waypoint <= final_wp:
                rospy.loginfo("decelerate to zero %s waypoints", len(final_waypoints.waypoints))
                final_waypoints.waypoints = self.decelerate(final_waypoints.waypoints)

        wps = [(wp.pose.pose.position.x,wp.pose.pose.position.y)
               for wp in final_waypoints.waypoints]
        speeds = [wp.twist.twist.linear.x for wp in final_waypoints.waypoints]
        rl_wp = self.waypoints[self.redlight_waypoint] if self.redlight_waypoint is not None else None
        rl_wp_coord = (rl_wp.pose.pose.position.x,rl_wp.pose.pose.position.x) if rl_wp is not None else (-1, -1)
        rospy.loginfo("final %s %s %s", rl_wp_coord,
                      len(wps), zip(wps, speeds))

        self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, msg):
        if (self.waypoints is None):
            self.waypoints = msg.waypoints
            rospy.loginfo("waypoints %s", len(self.waypoints))
        else:
            self.base_waypoints_sub.unregister()

    def closest_waypoint(self, waypoints, pose):
        def dl(a, b):
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

        closest_wp = len(waypoints)-1
        closest_dist = None
        # for i in range(len(waypoints)):
        #     dist = dl(waypoints[i].pose.pose.position, pose.position)
        #     if (dist < closest_dist):
        #         closest_wp = i
        #         closest_dist = dist

        wp_lower = 0
        wp_upper = len(waypoints) - 1
        while wp_lower < wp_upper:
            wp_mid = (wp_lower + wp_upper) // 2
            dist_lower = dl(waypoints[wp_lower].pose.pose.position, pose.position)
            dist_upper = dl(waypoints[wp_upper].pose.pose.position, pose.position)
            dist_mid = dl(waypoints[wp_mid].pose.pose.position, pose.position)

            closest_dist = dist_lower
            closest_wp = wp_lower
            if dist_mid < closest_dist:
                closest_dist = dist_mid
                closest_wp = wp_mid
            if dist_upper < closest_dist:
                closest_dist = dist_upper
                closest_wp = wp_upper

            rospy.logdebug("wp low %s %s mid %s %s up %s %s closest %s %s",
                           wp_lower, dist_lower,
                           wp_mid, dist_mid,
                           wp_upper, dist_upper,
                           closest_wp, closest_dist)

            # if all contiguous its converged closest_wp
            if wp_lower == wp_mid -1 and wp_mid == wp_upper -1:
                break

            # handle track looping around to start
            if dist_upper < dist_mid:
                if dist_lower < dist_upper:
                    wp_upper = wp_mid - 1
                else:
                    wp_lower = wp_mid + 1

            elif dist_mid < closest_dist:
                wp_lower = wp_mid-1
            elif closest_dist < dist_mid:
                wp_upper = wp_mid+1
            elif dist_lower < dist_upper:
                wp_upper = wp_mid + (wp_upper - wp_mid) // 2
            elif dist_upper < dist_lower:
                wp_lower = wp_mid - (wp_mid - wp_lower) // 2

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
        # rospy.loginfo("traffic_cb %s", msg)

        self.redlight_waypoint = msg.data if msg.data != -1 else None

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def decelerate(self, waypoints):
        def distance(p1, p2):
            x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
            return math.sqrt(x*x + y*y + z*z)

        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist) * 3.6
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

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
