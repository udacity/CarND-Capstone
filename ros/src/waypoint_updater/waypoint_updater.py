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

RATE = 2    # updtae rate
MAX_VEL = 3.

class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo('Initializing the base model') 
     
        rospy.init_node('waypoint_updater')

        ########### Self parameters  ###############

        self.base_waypoints = None            # base points coming from csv file                
        self.curr_pose = None                 # current pose
        self.final_waypoints =  None
        self.tree = None
        self.curr_velocity = None
        self.next_waypoint_index  = None
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        rospy.Subscriber('/current_velocity', TwistStamped, callback=self.currvel_cb)
#        self.base_wp = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        wp = rospy.wait_for_message('/base_waypoints', Lane)
        
        if not self.base_waypoints:
            self.base_waypoints = wp.waypoints  
            rospy.logwarn('Got the base points')        
        
        b_xcor = []
        b_ycor = []
        for pt in self.base_waypoints:
            b_xcor.append(pt.pose.pose.position.x)
            b_ycor.append(pt.pose.pose.position.y)
        self.tree= KDTree(zip(b_xcor, b_ycor))
        #rospy.logwarn('Got the base points  tot are %s',len(b_xcor))
        
        
        
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32 ,self.traffic_cb)
        
        #rospy.Subscriber('/obstacle_waypoint',,self.obstacle_cb)
        
        
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

#        rospy.spin()
        self.loop_n_sleep()
   
    def currvel_cb (self,msg):
        self.curr_velocity = msg.twist.linear.x
    def pose_cb(self, msg):
        # TODO: Implement
        self.curr_pose = msg.pose
        #pass




# Now defunt waypoint call back
#    def waypoints_cb(self, waypoints):
#        # TODO: Implement
#        if not self.base_waypoints:
#            self.base_waypoints = waypoints.waypoints  
#            rospy.logwarn('Got the base points')
#          
#    # Only get these points once    
#        if self.base_waypoints:   
#           self.base_wp.unregister() 
#   # get x and y points
#        b_xcor = []
#        b_ycor = []
#        for pt in self.base_waypoints:
#            b_xcor.append(pt.pose.pose.position.x)
#            b_ycor.append(pt.pose.pose.position.y)
#        self.tree= KDTree(zip(b_xcor, b_ycor))
#        rospy.logwarn('Got the base points  tot are %s',len(b_xcor))
 
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
  

    def loop_n_sleep(self):
        '''
        Should get current location with respect to base points, and draw next location path
        publish this path        
        and then sleep
        '''        
        rate = rospy.Rate(RATE)
        
        while not rospy.is_shutdown():
              #rospy.logwarn('Entering the publisher now')
          
              if self.base_waypoints and self.curr_pose:
                
                #rospy.logwarn('Entering the publisher now')
                # Get distance and index for the closest waypoint
                dist,idx = self.tree.query((self.curr_pose.position.x,self.curr_pose.position.y))
                #self.next_waypoint_index = idx
                
                # get actual point coordinates
                actual_wp_x = self.base_waypoints[idx].pose.pose.position.x
                actual_wp_y = self.base_waypoints[idx].pose.pose.position.y

                x,y = self.curr_pose.position.x, self.curr_pose.position.y
                #get orientation
                orient = math.atan2((actual_wp_y - y ),(actual_wp_x-x))
                angle = abs(0 - orient)
                #check if idx point is behind or in front of car
                if angle > math.pi / 4:
                    idx += 1
                
                #self.next_waypoint_index = self.ahead_waypoint(idx)
                self.next_waypoint_index = idx
                #rospy.logwarn('Closest index is %s', self.next_waypoint_index )                
                self.publish(self.next_waypoint_index   )
        
        
              rate.sleep()

    def publish(self,idx):

            # Create Lane object and set timestamp
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()
    
            # Update waypoints and set their velocities. 
        self.final_waypoints = deepcopy(self.base_waypoints[idx: idx + LOOKAHEAD_WPS])
#        for pt in self.final_waypoints:
#            rospy.logwarn('Next pointis %s %s ',pt.pose.pose.position.x, pt.pose.pose.position.y)
        
        # This segment should be replaced with actual speed values, for now speed is set to max
        if self.curr_velocity <= 1:
            speed = MAX_VEL
        else:
            speed  = MAX_VEL
        #########################
        for wp in self.final_waypoints:
            self.set_waypoint_velocity(wp, speed) #Accelele        
        

        final_waypoints_msg.waypoints = self.final_waypoints
        #rospy.logwarn('publishing points')
            # Publish results
        self.final_waypoints_pub.publish(final_waypoints_msg)





    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    
    
    
    #  Helper function to get car heading through tf package
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
