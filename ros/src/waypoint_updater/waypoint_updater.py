#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import tf

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. 


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        #The /base_waypoints topic repeatedly publishes a list of all waypoints for the track, 
        # so this list includes waypoints both before and after the vehicle. 
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below 
	self.current_pose = None
        self.current_waypoints = None #base_points which we get
	self.traffic_waypoint = None
	rospy.loginfo('Starting of waypoint updater')

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 40Hz
        while not rospy.is_shutdown():
            if ((self.current_pose is not None) and (self.current_waypoints is not None)):
                next_waypoint_index = self.get_next_waypoint()
                rospy.loginfo ("started")
		lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.current_waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]
                self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def get_nearest_waypoint(self):
        min_dist = 999999
        min_ind = 0
        ind = 0
        car_position = self.current_pose.pose.position
        rospy.loginfo ("car - position:", car_position)

        # Calcuate distance between car and waypoint
        for wp in self.current_waypoints:
            waypoint_position = wp.pose.pose.position 
            dist = self.get_euclidean_distance(car_position, waypoint_position)
            
            # Store the index of waypoint which is nearest to the car
            if dist < min_dist:
                min_dist = dist
                min_ind = ind
            ind += 1
        return min_ind


    def get_current_yaw(self):
        quaternion = (self.current_pose.pose.orientation.x)
        quaternion = (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw


    def get_next_waypoint(self):
        ind = self.get_nearest_waypoint()
        
        map_x = self.current_waypoints[ind].pose.pose.position.x
        map_y = self.current_waypoints[ind].pose.pose.position.y

        car_x = self.current_pose.pose.position.x
        car_y = self.current_pose.pose.position.y

        direction = math.atan2((map_y-car_y), (map_x-car_x)) #get direction
	rospy.loginfo ("direction:", direction)
        yaw = self.get_current_yaw()
        angle = abs(yaw - direction);

        if (angle > math.pi/4):
            ind += 1
	    
        return ind

    def pose_cb(self, msg):
        self.current_pose = msg
        pass

    def waypoints_cb(self, lane):
        self.current_waypoints = lane.waypoints;
	rospy.loginfo ("lane.waypoints:", lane.waypoints)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. 
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        pass
    # gets the linear velocity (x-direction) for a single waypoint.    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # Sets the linear velocity (x-direction) for a single waypoint in a list of waypoints. 
    # Here, waypoints is a list of waypoints, waypoint is a waypoint index in the list, and velocity is the desired velocity.    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2  + (pos1.z-pos2.z)**2)

    '''
    Computes the distance between two waypoints in a 
    list along the piecewise linear arc connecting all waypoints between the two. 
    Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list. 
    This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light 
    (the velocities should gradually decrease to zero starting some distance from the light).
	'''

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        #dl=0
        #dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += self.get_euclidean_distance(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
