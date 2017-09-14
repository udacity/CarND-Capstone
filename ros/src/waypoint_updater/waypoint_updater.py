#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
import numpy as np
import tf
import math
import std_msgs.msg
from std_msgs.msg import Bool, Float64, Int32
from scipy.interpolate import interp1d

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
DEBUG         = False

class WaypointUpdater(object):
    def __init__(self):
        self.cur_pose = None
        self.base_waypoints = None
        self.next_waypoints = None
        self.is_signal_red = False
        self.prev_pose = None
        self.move_car = False

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.is_signal_red_pub = rospy.Publisher('is_signal_red', Bool, queue_size=1)

        self.publish()

        rospy.spin()

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.cur_pose is not None) and (self.base_waypoints is not None):
                next_wp_i = self.next_waypoint(self.cur_pose.pose, self.base_waypoints.waypoints)
                if self.is_signal_red == True and \
                   self.red_wp_i and self.red_wp_i > next_wp_i:
                     red_wp_i = self.red_wp_i
                     sp_wp = [next_wp_i, red_wp_i]
                     next_wp_velocity = self.get_waypoint_velocity(self.base_waypoints.waypoints[next_wp_i])
        	     sp_v = [next_wp_velocity, 0.0]
                     f_sp = interp1d(sp_wp, sp_v)
	             for  p in range(next_wp_i, red_wp_i):
                    	self.set_waypoint_velocity(self.base_waypoints.waypoints,
                                                    next_wp_i,f_sp(p))
                     if self.red_wp_i:
		         self.set_waypoint_velocity(self.base_waypoints.waypoints, red_wp_i,0)
                     if DEBUG:
                         rospy.loginfo("set velocity to 0")
                elif self.move_car == True:
                     self.set_waypoint_velocity(self.base_waypoints.waypoints,next_wp_i, 4.5)
                     self.move_car = False
                next_waypoints = self.base_waypoints.waypoints[next_wp_i:next_wp_i+LOOKAHEAD_WPS]

                # publish
                final_waypoints_msg = Lane()
                final_waypoints_msg.header.frame_id = '/world'
                final_waypoints_msg.header.stamp = rospy.Time(0)
                final_waypoints_msg.waypoints = next_waypoints
                self.final_waypoints_pub.publish(final_waypoints_msg)

                self.is_signal_red_pub.publish(self.is_signal_red)

            rate.sleep()

    def pose_cb(self, msg):
        self.cur_pose = msg
                           
    def waypoints_cb(self, msg):
        self.base_waypoints = msg

    def traffic_cb(self, msg):
        if DEBUG:
            rospy.logerr('Got TL')
            rospy.logerr("message = %s", msg)

        if msg.data  >=  0: 
            self.is_signal_red = True
	    self.red_wp_i = msg.data
            if DEBUG:
                rospy.loginfo("data %s signal  = true", msg.data)
        else:
            if self.prev_pose == None:
                self.prev_pose = self.cur_pose
            else:
                if (self.prev_pose.pose.position.x == self.cur_pose.pose.position.x) and (self.prev_pose.pose.position.y == self.cur_pose.pose.position.y):
                    self.is_signal_red = False
                    self.red_wp_i = None
                    self.move_car = True
                    self.prev_pose = self.cur_pose
                    if DEBUG:
                        rospy.loginfo("velocity 0 hence changing the state")
                self.prev_pose = self.cur_pose

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp_idx, velocity):
        waypoints[wp_idx].twist.twist.linear.x = velocity
     

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint(self, pose, waypoints):
        closest_len = 100000
        closest_wp_i = 0
        dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2
        for i in range(len(waypoints)):
            dist = dl(pose.position, waypoints[i].pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_wp_i = i
        return closest_wp_i

    def next_waypoint(self, pose, waypoints):
        closest_wp_i = self.closest_waypoint(pose, waypoints)
        map_x = waypoints[closest_wp_i].pose.pose.position.x
        map_y = waypoints[closest_wp_i].pose.pose.position.y
        
        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))

        pose_quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (_, _, yaw) = tf.transformations.euler_from_quaternion(pose_quaternion)
        angle = math.fabs(heading - yaw)
        
        if DEBUG:
            rospy.logerr('current_pose - x:{}, y:{},z:{}'.format(pose.position.x, pose.position.y, pose.position.z))
            rospy.logerr('ego yaw: {}'.format(yaw))
            rospy.logerr('heading: {}, angle: {}'.format(heading, angle))
            rospy.logerr('closest wp: {}; {}-{}'.format(closest_wp_i, waypoints[closest_wp_i].pose.pose.position.x, waypoints[closest_wp_i].pose.pose.position.y))

        if angle > (math.pi / 4):
            closest_wp_i += 1
            if DEBUG:
                rospy.logerr('corrected wp: {}; {}-{}'.format(closest_wp_i, waypoints[closest_wp_i].pose.pose.position.x, waypoints[closest_wp_i].pose.pose.position.y))

        if DEBUG:
            rospy.logerr(' ')
        
        return closest_wp_i


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
