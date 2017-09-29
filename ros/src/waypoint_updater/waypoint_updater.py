#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
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
        rospy.init_node('waypoint_updater')
        
        self.current_pose = None
        self.base_waypoints = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
	rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
	# first version	
	if(self.current_pose and self.base_waypoints):
	    # get closest waypoint index
            closest_index = self.get_closest_waypoint(self.current_pose.pose)
            # get the first waypoint index currently ahead of the car 
            next_index = self.get_next_waypoint(self.current_pose.pose, closest_index)  
            # final_waypoints
            final_waypoints = []
            for i in range(next_index, next_index + LOOKAHEAD_WPS):
                i = i % len(self.base_waypoints.waypoints)
                p = self.base_waypoints.waypoints[i]
	        final_waypoints.append(p)
	    
	    self.publish(final_waypoints)

    def dist(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    # Carry over codes from CarND-Path-Planning-Project
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
	closest_index = 0
     	closest_dist = 100000
        p1 = pose.position
	wl = self.base_waypoints.waypoints
	
	for i in range(len(wl)):
	    p2 = wl[i].pose.pose.position
	    d = self.dist(p1, p2)
    	    if(d < closest_dist):
     	        closest_dist = d
                closest_index = i
	
        return closest_index

    def get_next_waypoint(self, pose, index):
	"""Identifies the first waypoint that is currently ahead of the car
        Args:
            index(int): index of the closest waypoint in self.waypoints

        Returns:
            int: index of the first waypoint currently ahead of the car

	"""
	next_index = index 
        p1 = pose.position
	p2 = self.base_waypoints.waypoints[index].pose.pose.position
	heading = math.atan2( (p2.y-p1.y),(p2.x-p1.x) );
	quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	angle = abs(yaw-heading);

        if angle > math.pi/4:
            next_index += 1
	
	return next_index

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_pose = msg
  
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
