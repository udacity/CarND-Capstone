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
MAX_DIST      = 50.0
BREAK_DIST    = 5.0
SLOW_VELOCITY = 1.5
FULL_VELOCITY = 4.5

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
        self.velocity_pub = rospy.Publisher('velocity_reference', Float64, queue_size=1)
        self.velocity_reference = 0.0

        self.tl_pos_x = None
        self.tl_pos_y = None
        self.tl_color = None 

       # rospy.Subscriber('/traffic_waypoint', TrafficLight, self.tl_cb)

        self.publish()

        rospy.spin()

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.cur_pose is not None) and (self.base_waypoints is not None):
                wp_base_size = len(self.base_waypoints.waypoints)
                # rospy.loginfo('wp_base_size: {}'.format(wp_base_size)) # 10902 wps at start
                next_wp_i = self.next_waypoint(self.cur_pose.pose, self.base_waypoints.waypoints)
                if self.is_signal_red == True:
                     rospy.loginfo("set velocity to 0")
                     self.set_waypoint_velocity(self.base_waypoints.waypoints,next_wp_i,0)
                elif self.move_car == True:
                     self.set_waypoint_velocity(self.base_waypoints.waypoints,next_wp_i,5)
                     self.move_car = False
                next_waypoints = self.base_waypoints.waypoints[next_wp_i:next_wp_i+LOOKAHEAD_WPS]

                # publish
                final_waypoints_msg = Lane()
                final_waypoints_msg.header.frame_id = '/world'
                final_waypoints_msg.header.stamp = rospy.Time(0)
                final_waypoints_msg.waypoints = next_waypoints
                self.final_waypoints_pub.publish(final_waypoints_msg)

                # Obtain light position
                if self.tl_color is not None:
                    x_ego   = self.cur_pose.pose.position.x
                    y_ego   = self.cur_pose.pose.position.y

                    pose_quaternion = (self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z, self.cur_pose.pose.orientation.w)
                    (_, _, yaw) = tf.transformations.euler_from_quaternion(pose_quaternion)
                    sin_yaw = math.sin(yaw)
                    cos_yaw = math.cos(yaw)

                    light_pos_vehicle = self.convert_ego_to_vehicle(x_ego, y_ego, cos_yaw, sin_yaw, self.tl_pos_x, self.tl_pos_y)

                    # Set reference velocity
                    if ( ((light_pos_vehicle[0] < MAX_DIST) and (light_pos_vehicle[0] >= BREAK_DIST)) and (self.tl_color == 0) ):
                        self.velocity_reference = SLOW_VELOCITY
                        if DEBUG:
                            rospy.logerr('Slow velocity')
                    elif ( ((light_pos_vehicle[0] < BREAK_DIST) and (light_pos_vehicle[0] > 0.0)) and (self.tl_color == 0) ):
                        self.velocity_reference = 0.0
                        if DEBUG:
                            rospy.logerr('Break')
                    else:
                        self.velocity_reference = FULL_VELOCITY
                        if DEBUG:
                            rospy.logerr('Full velocity')

                    self.velocity_pub.publish(self.velocity_reference)

            rate.sleep()

    def pose_cb(self, msg):
        self.cur_pose = msg
                           
    def waypoints_cb(self, msg):
        self.base_waypoints = msg

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo("message = %s", msg)
        if DEBUG:
            rospy.logerr('Got TL')
        if msg.data  >=  0: 
             self.is_signal_red = True
             rospy.loginfo("data %s signal  = true", msg.data)
             waypoint = msg.data
             #self.set_waypoint_velocity(self.base_waypoints.waypoints,waypoint,0)
        else:
             if self.prev_pose == None:
                    self.prev_pose = self.cur_pose
             else:
                    if self.prev_pose.pose.position.x == self.cur_pose.pose.position.x and self.prev_pose.pose.position.y == self.cur_pose.pose.position.y:
                         self.is_signal_red = False
                         rospy.loginfo("velocity 0 hence changing the state")
                         self.move_car = True
                    self.prev_pose = self.cur_pose
        
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

   # def tl_cb(self, msg):
   #     self.tl_pos_x = msg.pose.pose.position.x
   #     self.tl_pos_y = msg.pose.pose.position.y
   #     self.tl_color = msg.state
   #     if DEBUG:
   #         rospy.logerr('Got TL')

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

    def convert_ego_to_vehicle(self, x_ego, y_ego, cos_yaw, sin_yaw, p_x, p_y):
        x_trans = p_x - x_ego
        y_trans = p_y - y_ego

        x_veh   =  x_trans * cos_yaw + y_trans * sin_yaw 
        y_veh   = -x_trans * sin_yaw + y_trans * cos_yaw 

        return (x_veh, y_veh)

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
