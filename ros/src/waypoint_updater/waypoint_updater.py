#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf
from platform import dist

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
        self.base_waypoints_sub =rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # TODO: Add other member variables you need below
        self.next_wp_idx= -1
        self.cur_pose = None
        self.waypoints = None
        
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.cur_pose = msg.pose
        #rospy.loginfo("Current Pose: %s", self.cur_pose)
        
        self.next_wp_idx = self.get_next_waypoint_idx()
        #rospy.loginfo("Next Waypoint Idx: %s", self.next_wp_idx)
        next_waypoints = self.waypoints
        next_waypoints = self.waypoints[self.next_wp_idx:(self.next_wp_idx+LOOKAHEAD_WPS)]
        
        lane = Lane()
        lane.waypoints = next_waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)
        
        rospy.loginfo("Next Waypoint: %s \nCurrent Pose: %s", next_waypoints[0].pose.pose, self.cur_pose)
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #Samer: waypoints is always the same so only need to load once
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints# + waypoints.waypoints
            self.base_waypoints_sub.unregister()

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
    
    def get_vector_from_quaternion(self, q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z
    
    def get_next_waypoint_idx(self):
        
        if self.waypoints is None or self.cur_pose is None:
            return -1
        
        cur_x = self.cur_pose.position.x
        cur_y = self.cur_pose.position.y
        
        #finding closest waypoint
        num_wp = len(self.waypoints)/2 + 5
        min_dist = -1
        closest_wp_idx = -1
        for i in range(0, num_wp):
            wp_x = self.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints[i].pose.pose.position.y
            
            dist = math.sqrt((wp_x-cur_x)**2 + (wp_y-cur_y)**2)
            
            if min_dist == -1 or dist < min_dist:
                min_dist = dist
                closest_wp_idx = i
                
                
        #getting closest waypoint in front of vehicle
        while(1):
            closest_wp = self.waypoints[closest_wp_idx]
            closest_wp_x = closest_wp.pose.pose.position.x
            closest_wp_y = closest_wp.pose.pose.position.y
            
            x_vec, y_vec,z_vec = self.get_vector_from_quaternion(self.cur_pose.orientation)
            
            vec_dist =  math.sqrt((closest_wp_x-cur_x - x_vec*0.1)**2 + (closest_wp_y-cur_y - y_vec*0.1)**2)
            
            closest_wp_idx+=1
            
            if vec_dist > min_dist:
                break
            #else:
                #return closest_wp_idx + 1
        
        return closest_wp_idx    
            
        
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
