#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Waypoint, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx = None
        
        
        #based on instructiuon we want to loop with 50 herts
        self.update_rate = rospy.Rate(50)
        self.loop()
        
    def loop(self):
        while not rospy.is_shutdown():
            if self.pose and self.waypoints_2d:
             
                self.publish_final_waypoints()
            self.update_rate.sleep()
            
    def get_closest_waypoint_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        index = self.waypoints_tree.query([x,y],1)[1]
        closest_point = self.waypoints_2d[index]
        previous_point = self.waypoints_2d[index-1]
        
        #find closest in moving direction
        closest_point_vec = np.array(closest_point)
        prev_point_vec = np.array(previous_point)
        current_point = np.array([x,y])
        if np.dot(closest_point_vec-prev_point_vec,current_point-closest_point_vec)>0:
            index = (index+1)%len(self.waypoints_2d)
        return index
    
    def publish_final_waypoints(self,index):
        final_lane = generate_lane()
        self.final_waypoints_pub.publish(final_lane)
    
    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header
        closest_idx = self.get_closest_waypoint_index()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx :
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_idx)
        return lane
        
    def decelerate_waypoints(self, waypoints, closest_idx): 
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(closest_idx-self.stopline_wp_idx-2,0) # stop before the before the stopline index
            distance_to_stop = distance(waypoints,i,stop_idx)
            velocity = math.sqrt(2*MAX_DECEL*distance_to_stop)
            if velocity<1:
                velocity = 0
            p.twist.twist.linear.x = min(velocity,p.twist.twist.linear.x)
            temp.append(p)
        return temp
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose=msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement

        self.base_waypoints = waypoints
        if not self.base_waypoints:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.stopline_wp_idx = msg.data

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


