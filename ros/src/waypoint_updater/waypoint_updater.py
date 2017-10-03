#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf

import math
import PyKDL
from scipy.interpolate import interp1d
from copy import deepcopy

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


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.waypoints = None
        self.waypoints_header = None

        rospy.spin()

    def pose_cb(self, msg):
        """
        msg:

        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        """
        pose = msg.pose
        pos = pose.position
        quat = PyKDL.Rotation.Quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)
        orient = quat.GetRPY()
        yaw = orient[2]
        # rospy.loginfo("xy: {}, {}".format(pos.x, pos.y))
        # rospy.loginfo("orient: {}".format(orient))

        if self.waypoints is not None:
            cur_wp_id = self.closest_waypoint(pos, yaw)
            # rospy.loginfo("cur_wp_id: {}".format(cur_wp_id))
            wp_x = []
            wp_y = []
            lane = Lane()
            for wp in self.waypoints[cur_wp_id:(cur_wp_id+LOOKAHEAD_WPS)]:
                x = wp.twist.twist.linear.x
                y = wp.twist.twist.linear.y

                wp_x.append(x)
                wp_y.append(y)
                lane.waypoints.append(deepcopy(wp))

            spline_xy = interp1d(wp_x, wp_y)

            rospy.loginfo("About to publish to final_waypoints")
            self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.waypoints_header = waypoints.header

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

    def closest_waypoint(self, position, yaw):
        """ Find the closest waypoint from a given pose
        
        Args:
            position (geometry_msgs/Point): Position from pose returned from `pose_cb()` method.
            yaw (float): The car's yaw.

        Return:
            int: ID of waypoint.
        """

        pos_threshold = 0.01
        yaw_threshold = 0.01

        min_dist = 10.0
        min_id = None

        # TODO: May need to find a faster way to find closest waypoint.
        for idx, wp in enumerate(self.waypoints):
            pos_dist = self.pos_distance(wp.pose.pose.position,
                                    position)
            quat = PyKDL.Rotation.Quaternion(wp.pose.pose.orientation.x,
                                             wp.pose.pose.orientation.y,
                                             wp.pose.pose.orientation.z,
                                             wp.pose.pose.orientation.w)
            orient = quat.GetRPY()

            yaw_dist = abs(orient[2] - yaw)
            comb_dist = (pos_dist + yaw_dist)
            if (pos_dist <= pos_threshold and yaw_dist <= yaw_threshold):
                return idx
            else:
                if comb_dist < min_dist:
                    min_dist = comb_dist
                    min_id = idx

        return min_id

    def pos_distance(self, a, b):
        """ Distance between two positions
        """
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def trycall():
        return 1;

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
