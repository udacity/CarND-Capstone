#!/usr/bin/env python
import sys
sys.path.append('/home/student/work/ros/src/styx_msgs/')
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Quaternion#

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

RATE = 2    # update rate
MAX_VEL = 3.   # max velocity in mps

class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo('Initializing the base model') 
     
        rospy.init_node('waypoint_updater')

        ########### Self parameters  ###############

        self.base_waypoints = None                                  # base points coming from csv file                
        self.curr_pose = None                                       # current pose
        self.final_waypoints =  None                                # final waypoints to publish for other nodes
        self.tree = None                                            # tree struct for coordinates
        self.curr_velocity = None                                   # current velocity    
        self.next_waypoint_index  = None                            # Index of the first waypoint in front of the car
        self.velocity =  rospy.get_param("/waypoint_loader/velocity", MAX_VEL)      # Max. velocity from gotten from ros parameters
        
        # if we get value from ros, convert it from km/h to meter per second (mps)
        if (self.velocity != MAX_VEL): 
            self.velocity = (self.velocity * 1000) / 3600 
        
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
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
                
                # Publish the next waypoints the car should follow
                self.publish(self.next_waypoint_index)
        
            rate.sleep()

    # Publish the next waypoints the car should follow
    def publish(self,idx):

        # Create Lane object and set timestamp
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()
    
        # Update waypoints and set their velocities. 
        self.final_waypoints = deepcopy(self.base_waypoints[idx: idx + LOOKAHEAD_WPS])

        # for pt in self.final_waypoints:
        #   rospy.logwarn('Next pointis %s %s ',pt.pose.pose.position.x, pt.pose.pose.position.y)
        
        # This segment should be replaced with actual speed values, for now speed is set to max
        if self.curr_velocity <= 1:
            speed = self.velocity
        else:
            speed  = self.velocity
        #########################
        
        # Set the velocity in x direction (car coordinate system) for the waypoints to follow
        for wp in self.final_waypoints:
            self.set_waypoint_velocity(wp, speed) #Accelele        
        
        # Set waypoints in waypoint message
        final_waypoints_msg.waypoints = self.final_waypoints
        
        #rospy.logwarn('publishing points')
            # Publish results
        
        # Publish the waypoints the car should follow to ros topic '/final_waypoints'
        self.final_waypoints_pub.publish(final_waypoints_msg)


    # Function to get the distance between two waypoints
    # TODO: not used at the moment
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Helper function to get car heading through tf package
    # TODO: Not used at the moment 
    def get_euler_yaw(self):
        quaternion = (
            self.curr_pose.orientation.x,
            self.curr_pose.orientation.y,
            self.curr_pose.orientation.z,
            self.curr_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]    
 

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
