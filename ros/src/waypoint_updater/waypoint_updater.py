#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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
        self.waypoints_2D = None
        self.waypoints_tree = None
        self.base_waypoints = None
        self.current_position = None
        self.total_waypoints_count = 0
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', PoseStamped, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # rospy.spin()
        self.loop()

    def pose_cb(self, msg):
        # rospy.logdebug("pose_cb!!")
        self.current_position = msg
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2D:
            rospy.logdebug("waypoints_cb!!")
            self.waypoints_2D=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree=KDTree(self.waypoints_2D)
            self.total_waypoints_count = len(self.waypoints_2D)
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

    def loop(self):
        rate=rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.current_position and self.base_waypoints:
                self.publish_finaly_waypoints()
            rate.sleep()

    def publish_finaly_waypoints(self):
        lane = Lane()
        car_x=self.current_position.pose.position.x
        car_y=self.current_position.pose.position.y
        if self.waypoints_tree:
            closest_indx=self.waypoints_tree.query([car_x,car_y],1)[1]
            closest_cord=self.waypoints_2D[closest_indx]
            previous_cord=self.waypoints_2D[(closest_indx-1)% self.total_waypoints_count]

            cl_vect=np.array(closest_indx)
            prev_vect=np.array(closest_indx-1)
            pos_vect=np.array([car_x,car_y])
            val=np.dot(cl_vect-prev_vect,pos_vect-cl_vect)

            if val.all()>0:
                closest_indx=(closest_indx+1) % self.total_waypoints_count
            lane.header=self.base_waypoints.header
            lane.waypoints=self.base_waypoints.waypoints[closest_indx:closest_indx+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
