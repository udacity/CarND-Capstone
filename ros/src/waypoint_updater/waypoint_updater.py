#!/usr/bin/env python
import sys
sys.path.append('/home/student/work/ros/src/styx_msgs/')
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Quaternion

from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import tf
import math
from copy import deepcopy
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
ACC_WPS_NUM = 10 # Number of waypoints used for acceleration

RATE = 2    # update rate
MAX_VEL = 3.   # max velocity in mps

class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo('Initializing the base model') 
     
        rospy.init_node('waypoint_updater')

        ########### Self parameters  ###############

        self.base_waypoints = None                                  # base points coming from csv file                
        self.curr_pose = None                                       # current pose
        self.traffic_pose = None                                    # position of the next traffic light (ground truth)
        self.traffic_status = None                                  # status of the next traffic light (ground truth)
        self.final_waypoints =  None                                # final waypoints to publish for other nodes
        self.tree = None                                            # tree struct for coordinates
        self.curr_velocity = None                                   # current velocity    
        self.max_velocity = None				                        # Value for max velocity        
        self.next_waypoint_index  = None                             # Index of the first waypoint in front of the car
        self.traffic_index = None
        self.max_velocity =  rospy.get_param("/waypoint_loader/velocity", MAX_VEL)      # Max. velocity from gotten from ros parameters
        
        # if we get value from ros, convert it from km/h to meter per second (mps)
        if (self.max_velocity != MAX_VEL): 
            self.max_velocity = (self.max_velocity * 1000) / 3600 
        
        # Subscribe to topic '/current_pose' to get the current position of the car
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # Subscribe to topic '/current_velocity' to get the current velocity of the car
        rospy.Subscriber('/current_velocity', TwistStamped, callback=self.currvel_cb)

        # Fetch info from '/base_waypoints', which provide a complete list of waypoints the car will be following
        wp = rospy.wait_for_message('/base_waypoints', Lane)
        
        # Set the waypoints only once
        if not self.base_waypoints:
            self.base_waypoints = wp.waypoints  
            rospy.logwarn('Got the base points')        
        
        # Get the x/y coordinates of the base_waypoints
        b_xcor = []
        b_ycor = []
        
        for pt in self.base_waypoints:
            b_xcor.append(pt.pose.pose.position.x)
            b_ycor.append(pt.pose.pose.position.y)
        self.tree= KDTree(zip(b_xcor, b_ycor))
        
        #rospy.logwarn('Got %s base waypoints!',len(b_xcor))
        
        # Subscribe to topic '/current_pose' to get the locations to stop for red traffic lights
        rospy.Subscriber('/traffic_waypoint', Int32 ,self.traffic_cb)
        
        # TODO: Include if obstacle detection implementation is included later on
        #rospy.Subscriber('/obstacle_waypoint',,self.obstacle_cb)
        
        # Set the publisher to write messages to topic '/final_waypoints'
        # The next waypoints the car has to follow are published
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # rospy.spin()
        self.loop_n_sleep()


   
    ### Begin: Callback functions for subsribers to ROS topics

    # Callback function to set the current velocity of the car
    # Information provided by ros topic '/current_velocity'
    def currvel_cb(self,msg):
        self.curr_velocity = msg.twist.linear.x
    
    # Callback function to set the current position of the car
    # Information provided by ros topic '/current_pose'
    def pose_cb(self, msg):
        self.curr_pose = msg.pose

    # Callback function to get the locations to stop for red traffic lights
    # Information provided by ros topic '/current_velocity'
    def traffic_cb(self, msg):
        self.traffic_index = msg.data

    # Callback function to get the position of obstacles
    # Information provided by ros topic '/obstacle_waypoint'
    # TODO: Subsriber not used at the moment (see __init__)
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    ### End: Callback functions for subsribers to ROS topics


 
    # Function to get the velocity in x direction (car coordinate system) of a single waypoint
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # Function to set the velocity in x direction (car coordinate system) of a single waypoint
    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity


    # Function to get the current location with respect to base points, 
    # and draw next location path, publish this path and then sleep
    def loop_n_sleep(self):
        # Set the update rate
        rate = rospy.Rate(RATE)
        
        # Run as long as ros node is not shut down
        while not rospy.is_shutdown():
              #rospy.logwarn('Entering the publisher now')
          
            if self.base_waypoints and self.curr_pose:
                #rospy.logwarn('Entering the publisher now')
                
                # Get the distance and index of the closest waypoint (in front of the car)
                dist,idx = self.tree.query((self.curr_pose.position.x,self.curr_pose.position.y))
                
                # Get x/y coordinates of the closest waypoint
                actual_wp_x = self.base_waypoints[idx].pose.pose.position.x
                actual_wp_y = self.base_waypoints[idx].pose.pose.position.y

                # Get current x/y coordinates of the own car
                x = self.curr_pose.position.x
                y = self.curr_pose.position.y
                
                # Calculate the orientation and the angle
                orient = math.atan2((actual_wp_y - y ),(actual_wp_x-x))
                angle = abs(0 - orient)
                
                # Check if idx point is behind or in front of car
                # If behind take the next waypoint,
                # to get the closest waypoint which is in front of the car
                if angle > math.pi / 4:
                    idx += 1
                
                ## Set the index of the closest waypoint in front of the car
                self.next_waypoint_index = idx
                #rospy.logwarn('Closest index is %s', self.next_waypoint_index )                
                
                # Get the values for velocities for the upcoming waypoints
                value_waypoint_velocities = self.get_waypoint_velocities()

                # Publish the next waypoints the car should follow
                self.publish(self.next_waypoint_index, value_waypoint_velocities)
        
            rate.sleep()

    # Publish the next waypoints the car should follow
    def publish(self,idx, value_waypoint_velocities):

        # Create Lane object and set timestamp
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()
    
        # Update waypoints and set their velocities. 
        self.final_waypoints = deepcopy(self.base_waypoints[idx: idx + LOOKAHEAD_WPS])

        # for pt in self.final_waypoints:
        #   rospy.logwarn('Next pointis %s %s ',pt.pose.pose.position.x, pt.pose.pose.position.y)
        
        #rospy.logwarn(value_waypoint_velocities[0]*2.23694)
        
        for i in range(LOOKAHEAD_WPS):
            self.set_waypoint_velocity(self.final_waypoints[i], value_waypoint_velocities[i])      
        
        # Set waypoints in waypoint message
        final_waypoints_msg.waypoints = self.final_waypoints
        
        #rospy.logwarn('publishing points')
            # Publish results
        
        # Publish the waypoints the car should follow to ros topic '/final_waypoints'
        self.final_waypoints_pub.publish(final_waypoints_msg)

    def get_waypoint_velocities(self):
        # Array for waypoint velocities
        waypoint_velocities = []

        
        # Traffic light to stop at waypoint index traffic_index
        if self.traffic_index > 0:
            # Calculate the difference between current speed and final target speed
            diff_index = self.traffic_index - self.next_waypoint_index

            # Calculate how much the velocity should be reduced per waypoint
            diff_velocity = self.curr_velocity / (diff_index + 1)

            new_velocity = self.curr_velocity

            for i in range(LOOKAHEAD_WPS):
                # Before traffic sign
                if i <= diff_index:
                    new_velocity -= diff_velocity
                    if new_velocity < 0.1:
                        new_velocity = 0
                    
                    waypoint_velocities.append(new_velocity)
                # After traffic sign
                else:
                    waypoint_velocities.append(0)

        # No traffic light to stop for -> accelerate until speed limit is reached
        else:
            # Check if current velocity is smaller than max_velocity
            if self.curr_velocity <= self.max_velocity: 
                # Calculate how much the velocity should be raised per waypoint
                diff_velocity = (self.max_velocity - self.curr_velocity) / ACC_WPS_NUM
            # If too fast -> reduce speed
            else:
                diff_velocity = self.curr_velocity - self.max_velocity

            new_velocity = self.curr_velocity

            for i in range(LOOKAHEAD_WPS):
                # Before reaching max_velocity
                if i <= ACC_WPS_NUM:
                    new_velocity += diff_velocity
                    
                    waypoint_velocities.append(new_velocity)
                # After traffic sign
                else:
                    waypoint_velocities.append(self.max_velocity)

        return waypoint_velocities

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
