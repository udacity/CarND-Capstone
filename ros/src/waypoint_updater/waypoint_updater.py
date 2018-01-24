#!/usr/bin/env python

import sys
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        self.num_base_waypoints = None
        self.rate = 50 #Hz

        self.run()
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        if self.base_waypoints is None:
            self.num_base_waypoints = len(waypoints.waypoints)
            self.base_waypoints = waypoints.waypoints

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

    def pnt_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.pnt_dist(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def vector_direction(self, ref, a, b):
        # 1. calculate the difference of vectors a/b to the ref vector
        ax = a.x - ref.x
        ay = a.y - ref.y
        az = a.z - ref.z

        bx = b.x - ref.x
        by = b.y - ref.y
        bz = b.z - ref.z

        # 2. take the scalar product of the new vectors
        result = ax * bx + ay * by + az * bz

        # 3. if the result is positive, both point into the same direction
        is_same_dir = result > 0

        return is_same_dir

    def find_closest_waypoint(self):
        pose = self.current_pose # fix current pose
        min_dist = sys.maxsize # set to large number
        pnt_idx = 0

        for i in range(self.num_base_waypoints):
            dist = self.pnt_dist(pose.position, self.base_waypoints[i].pose.pose.position)
            if dist < min_dist:
                pnt_idx = i
                min_dist = dist

        # check if car has passed the closest waypoint already
        # or if it is the next one
        next_pnt_idx = (pnt_idx + 1) % self.num_base_waypoints

        # if the closest and the following waypoint point into the same
        # direction then the closest one the one in front
        # otherwise it is behind the vehicle and we use the following
        is_same_dir = self.vector_direction(pose.position,
                self.base_waypoints[pnt_idx].pose.pose.position,
                self.base_waypoints[next_pnt_idx].pose.pose.position)

        if is_same_dir:
            return pnt_idx, True
        else:
            return next_pnt_idx, False



    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.base_waypoints is not None and self.current_pose is not None:
                closest_idx = self.find_closest_waypoint()[0]

                # create list of next waypoints
                waypoints = []
                for i in range(LOOKAHEAD_WPS):
                    waypoint_id = (closest_idx + i) % self.num_base_waypoints
                    waypoints.append(self.base_waypoints[waypoint_id])

                rospy.loginfo(self.find_closest_waypoint())

                # construct message
                lane_msg = Lane()
                lane_msg.waypoints = waypoints

                self.final_waypoints_pub.publish(lane_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
