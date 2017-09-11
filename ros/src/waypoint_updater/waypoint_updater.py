#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
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

        self.current_pose = []
        self.tl_pose = []
        self.tl_state = 'green'

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', ??? , obstacle_cb)   #BUG - there is no obstacle_waypoint

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # todo: create a message of type Lane
            #lane_msg.header
            #lane_msg.waypoints

            # todo: get the current pose

            # todo: build a new set of waypoints by using only a given number (LOOKAHEAD_WPS)
            # todo: of base waypoints from the current pose onwards

            #self.final_waypoints_pub.publish(lane_msg)
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        # self.current_pose = msg
        # std_msgs / Header        header
        # uint32        seq
        # time        stamp
        # string        frame_id

        # geometry_msgs / Pose    pose
        # geometry_msgs / Point    position
        # float64    x
        # float64    y
        # float64    z

        # geometry_msgs / Quaternion orientation
        # float64 x
        # float64 y
        # float64 z
        # float64 w
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement

        # Lane msg description:
        #
        #     std_msgs / Header        header
        #     uint32        seq
        #     time        stamp
        #     string        frame_id
        #
        #     styx_msgs / Waypoint[]        waypoints
        #     geometry_msgs / PoseStamped        pose
        #     std_msgs / Header        header
        #     uint32        seq
        #     time        stamp
        #     string        frame_id
        #
        #
        #     geometry_msgs / Pose        pose
        #     geometry_msgs / Point        position
        #     float64        x
        #     float64        y
        #     float64        z
        #     geometry_msgs / Quaternion        orientation
        #     float64        x
        #     float64        y
        #     float64        z
        #     float64        w
        #     geometry_msgs / TwistStamped        twist
        #     std_msgs / Header        header
        #     uint32        seq
        #     time        stamp
        #     string        frame_id
        #     geometry_msgs / Twist        twist
        #     geometry_msgs / Vector3        linear
        #     float64        x
        #     float64        y
        #     float64        z
        #     geometry_msgs / Vector3        angular
        #     float64        x
        #     float64        y
        #     float64        z
        pass

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
