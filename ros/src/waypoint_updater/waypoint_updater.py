#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        self.track_waypoints = None

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        rospy.spin()

    def pose_cb(self, msg):
        if self.track_waypoints is None:
            return

        # todo: no need to recompute final_waypoints on every call

        waypoints = self.track_waypoints.waypoints
        car_point = msg.pose.position
        next_waypoint = WaypointUpdater.next_waypoint(waypoints, car_point)

        lane = Lane()
        for j in range(next_waypoint, next_waypoint + LOOKAHEAD_WPS):
            lane.waypoints.append(waypoints[j])

        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        if self.track_waypoints is None:
            self.track_waypoints = waypoints

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

    @staticmethod
    def distance(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def total_distance(waypoints, wp1, wp2):
        dist = 0

        for i in range(wp1, wp2+1):
            dist += WaypointUpdater.distance(
                waypoints[wp1].pose.pose.position,
                waypoints[i].pose.pose.position)
            wp1 = i

        return dist

    @staticmethod
    def closest_waypoint(waypoints, car_point):
        next_waypoint_index = 0
        next_waypoint_dist = 99999

        for i, waypoint in enumerate(waypoints):
            dist = WaypointUpdater.distance(
                waypoint.pose.pose.position,
                car_point)

            if dist < next_waypoint_dist:
                next_waypoint_index = i
                next_waypoint_dist = dist

        return next_waypoint_index

    @staticmethod
    def next_waypoint(waypoints, car_point):
        next_waypoint_index = WaypointUpdater.closest_waypoint(waypoints, car_point)

        next1 = waypoints[next_waypoint_index].pose.pose.position
        next2 = waypoints[next_waypoint_index + 1].pose.pose.position

        theta = math.atan2(next2.y - next1.y, next2.x - next1.x)
        heading = math.atan2(next1.y - car_point.y, next1.x - car_point.x)
        angle = abs(theta - heading)

        if angle > math.pi / 4.0:
            next_waypoint_index += 1

        return next_waypoint_index

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
