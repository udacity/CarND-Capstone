#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32


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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
KPH_TO_MPS = 1.0/3.6

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_velocity = None
        # get current velocity too
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)


        # TODO: Add other member variables you need below

        self.base_waypoints = None
        self.num_waypoints = -1

        self.pose = None
        self.pose_stamp = None

        self.car_x = None
        self.car_y = None

        self.car_theta = 0
        self.next_wp_index  = 0

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity')

        # Flags
        self.flag_waypoints_loaded = False
        self.STOPPING_DISTANCE = 15.
        self.traffic_waypoint_index = -1
        rospy.logout("self.STOPPING_DISTANCE = %f"%(self.STOPPING_DISTANCE))
        self.loop()
        #rospy.spin()

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            
            if self.flag_waypoints_loaded and self.pose is not None:
                
                self.car_x = self.pose.position.x
                self.car_y = self.pose.position.y

                # Now get the next waypoint....
                if self.flag_waypoints_loaded:
                    self.next_wp_index = self.findNextWaypoint()
                    
                    msg = Lane()
                    msg.waypoints = []

                    start_index = self.next_wp_index

                    current_velocity = self.current_velocity.linear.x if self.current_velocity is not None else 0.0
                    road_inex = start_index
                    
                    for i in range(LOOKAHEAD_WPS):
                        # index of the trailing waypoints 
                        wp = Waypoint()
                        wp.pose.pose.position.x = self.base_waypoints[road_inex].pose.pose.position.x
                        wp.pose.pose.position.y = self.base_waypoints[road_inex].pose.pose.position.y

                        if self.traffic_waypoint_index < len(self.base_waypoints) and self.traffic_waypoint_index > start_index:
                            # We have red head of front
                            
                            thisDistance = self.distance(self.base_waypoints, self.traffic_waypoint_index, road_inex)
                            
                            if (thisDistance > self.STOPPING_DISTANCE):
                                wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS
                            else:
                                
                                wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS * (float(thisDistance) / float(self.STOPPING_DISTANCE))
                        else:
                            wp.twist.twist.linear.x = self.speed_limit * KPH_TO_MPS

                        msg.waypoints.append(wp)
                        road_inex = (road_inex + 1) % self.num_waypoints

                    self.final_waypoints_pub.publish(msg)
            
    def pose_cb(self, msg):
        self.pose = msg.pose
        # get the time stamp. might be useful to calculate latency
        self.pose_stamp = msg.header.stamp

    def waypoints_cb(self, waypoints):
        if not self.flag_waypoints_loaded:
            self.base_waypoints = waypoints.waypoints
            self.num_waypoints = len(self.base_waypoints)
            self.flag_waypoints_loaded = True 

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_index = msg.data
        rospy.logdebug("waypoint_updater:traffic_cb says there is a red light at waypoint %s" , self.traffic_waypoint_index )
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    
    def current_velocity_cb(self, msg):
        ''' Callback for /current_velocity topic
            Simply save the current velocity value
        '''
        self.current_velocity = msg.twist

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def findNextWaypoint(self):
       
        nn_index = 0
        map_x = self.base_waypoints[nn_index].pose.pose.position.x
        map_y = self.base_waypoints[nn_index].pose.pose.position.y
        mindist = (self.car_x - map_x) ** 2 + (self.car_y - map_y) ** 2
        
        for i in range(1, self.num_waypoints):
            x = self.base_waypoints[i].pose.pose.position.x
            y = self.base_waypoints[i].pose.pose.position.y
            
            dist = (self.car_x - x) ** 2 + (self.car_y - y) ** 2            
            if (dist < mindist):
                mindist = dist
                nn_index = i
        
        next_wp_index = ( nn_index + 1 ) % len(self.base_waypoints)
        
        vx = self.base_waypoints[next_wp_index].pose.pose.position.x - map_x
        vy = self.base_waypoints[next_wp_index].pose.pose.position.y - map_y
        norm_v = np.sqrt( vx*vx + vy*vy )
        vx /= norm_v
        vy /= norm_v
        # now the difference : car position - nearest wp
        dx = self.car_x - map_x
        dy = self.car_y - map_y
        # Get the dot product of d and v
        dot = vx * dx + vy * dy
        if dot >= 0:
            return (next_wp_index)
        else:
            return (nn_index)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
