#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import sys
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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None
        self.len_waypoints = None

        rospy.spin()

    def check_is_behind(self, index):
        dx = self.base_waypoints.waypoints[index].pose.pose.position.x - self.pose.position.x
        dy = self.base_waypoints.waypoints[index].pose.pose.position.y - self.pose.position.y
        wp_angle = None

        if (dy == 0):
            if (dx >= 0):
                wp_angle = 0.5 * math.pi
            else:
                wp_angle = 1.5 * math.pi
        elif (dx >= 0.0 and dy > 0.0):
            wp_angle = math.atan(dx / dy)
        elif (dx >= 0.0 and dy < 0.0):
            wp_angle = math.pi - math.atan(-dx / dy)
        elif (dx < 0.0 and dy < 0.0):
            wp_angle = math.pi + math.atan(dx / dy)
        else:
            wp_angle = 2 * math.pi - math.atan(-dx / dy)

        # Normalize car's angle?
        delta_angle = abs(wp_angle - self.pose.orientation.w)
        if (delta_angle >= 0.5 * math.pi and delta_angle <= 1.5 * math.pi):
            return True
        else:
            return False

    def squared_dist(self, index):
        dx = self.pose.position.x - self.base_waypoints.waypoints[index].pose.pose.position.x
        dy = self.pose.position.y - self.base_waypoints.waypoints[index].pose.pose.position.y
        return dx**2 + dy**2

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        rospy.loginfo("Received new position: x={}, y={}".format(self.pose.position.x,
                                                                 self.pose.position.y))
        if (self.base_waypoints == None):
            rospy.loginfo("Cannot process new position, because waypoints are not yet loaded.")
            return

        closest_dist = float("inf")
        closest_index = 0

        for i in range(0, self.len_waypoints):
            curr_dist = self.squared_dist(i)
            if (curr_dist < closest_dist):
                closest_dist = curr_dist
                closest_index = i

        if (self.check_is_behind(closest_index)):
            closest_index += 1
            if (closest_index == self.len_waypoints):
                closest_index = 0

        rospy.loginfo("Closed Waypoint index is: {}, x={}, y={}".
                      format(closest_index,
                             self.base_waypoints.waypoints[closest_index].pose.pose.position.x,
                             self.base_waypoints.waypoints[closest_index].pose.pose.position.y))

        final_waypoints = None
        if (closest_index < self.len_waypoints - LOOKAHEAD_WPS):
            final_waypoints = self.base_waypoints.waypoints[closest_index:closest_index + LOOKAHEAD_WPS]
        else:
            final_waypoints = self.base_waypoints.waypoints[closest_index:]
            rest = LOOKAHEAD_WPS - (self.len_waypoints - closest_index)
            final_waypoints += self.base_waypoints.waypoints[:rest]

        rospy.loginfo("Length of final_waypoints is {}".format(len(final_waypoints)))
        assert (len(final_waypoints) == LOOKAHEAD_WPS)

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.header.frame_id = "/world"
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)

        rospy.logwarn("Published final waypoints...")
        rospy.loginfo(lane)


    def waypoints_cb(self, waypoints):
        if (self.base_waypoints == None):
            self.base_waypoints = waypoints
            self.len_waypoints = len(self.base_waypoints.waypoints)
            rospy.loginfo("Waypoints loaded... found {}.".format(self.len_waypoints))

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
