#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg import Int32, Bool
import math
import copy
import tf
from scipy.interpolate import CubicSpline

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
waypoints_search_range = 10  # Number of waypoints to search current position back and forth

MAX_DECEL = 1.

class WaypointUpdater(object):
    def __init__(self):
        # Initialize the Nodes
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.next_wp_pub = rospy.Publisher('/next_wp', Int32, queue_size=1)
        # Store the Waypoint List when Base WayPoint is Called 
        self.waypoints = None
        self.current_pose = None
        self.next_waypoint_index = None
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.waypoints and self.next_waypoint_index != None):
                # For now a very simple implementation to get things moving. Find and then Publish the Lookahead waypoints
                lane = Lane()
                lane.header.frame_id = self.current_pose.header.frame_id
                lane.header.stamp = rospy.Time.now()
                # Publish Waypoints for the lookahead defined
                if self.next_waypoint_index+LOOKAHEAD_WPS < len(self.waypoints):
                    lane.waypoints = self.waypoints[self.next_waypoint_index:self.next_waypoint_index+LOOKAHEAD_WPS]
                else:
                    lane.waypoints = self.waypoints[self.next_waypoint_index:]
                    lane.waypoints.extend(self.waypoints[0:(self.next_waypoint_index+LOOKAHEAD_WPS) % len(self.waypoints)])
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

  
    def distance_between_two_points(self, position1, position2):
        a = position1
        b = position2
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)



    # Find waypoint coordinates in Car Coordinate Systems
    def waypoint_in_car_coordinate_system(self, closest_waypoint_index):
        # Find World Coordinates for Waypoint
        world_closest_waypoint_x = self.waypoints[closest_waypoint_index].pose.pose.position.x
        world_closest_waypoint_y = self.waypoints[closest_waypoint_index].pose.pose.position.y
        # Find Car Coordinates
        car_coordinate_x = self.current_pose.pose.position.x
        car_coordinate_y = self.current_pose.pose.position.y
        # Find Yaw value for the car
        pitch, roll, yaw = tf.transformations.euler_from_quaternion((
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w))
        # Map the closest Waypoint in Car Coordinate System
        # Return as (x, y) tuple
        closest_waypoint_in_car_coordinate_system = ((world_closest_waypoint_x-car_coordinate_x) * math.cos(yaw), (world_closest_waypoint_y-car_coordinate_y) * math.sin(yaw))
        return closest_waypoint_in_car_coordinate_system

    # Update to the closest waypoint of your current position
    def identify_next_waypoint_index(self):
        # Get the Closest Waypoint to Current Car Position
        closest_waypoint_index = self.find_close_waypoint_in_range()
        # Find Map Coordinates for this WayPoint
        closest_waypoint_in_car_coordinate_system = self.waypoint_in_car_coordinate_system(closest_waypoint_index)
        # If Behind increase the waypoint index 
        if ( closest_waypoint_in_car_coordinate_system[0] < 0.):
            closest_waypoint_index += 1
        self.next_waypoint_index = closest_waypoint_index % len(self.waypoints)
        return closest_waypoint_index

    def find_search_range(self):
        search_range_begin_index = 0
        search_range_end_index = len(self.waypoints)
        if (self.next_waypoint_index):
            search_range_begin_index = max (self.next_waypoint_index - waypoints_search_range, 0)
            search_range_end_index = min (self.next_waypoint_index + waypoints_search_range, search_range_end_index)
        return search_range_begin_index, search_range_end_index
    
    def find_shortest_distance_waypoint_bet_range(self, search_range_begin_index, search_range_end_index):
        minimum_distance_value = 90000
        closest_waypoint_index = 0
        car_current_position = self.current_pose.pose.position
        for idx in range(search_range_begin_index, search_range_end_index):
            car_future_position = self.waypoints[idx].pose.pose.position
            dist = self.distance_between_two_points(car_current_position, car_future_position)
            if dist < minimum_distance_value:
                minimum_distance_value = dist
                closest_waypoint_index = idx
        return closest_waypoint_index

    # Helper method 
    def find_close_waypoint_in_range(self):       
        search_range_begin_index, search_range_end_index = self.find_search_range()
        return self.find_shortest_distance_waypoint_bet_range(search_range_begin_index, search_range_end_index)

    # This is where the real magic happens of moving the car. Find the closes waypoint and publish it
    def pose_cb(self, msg):
        self.current_pose = msg
        if self.waypoints:
            next_waypoint_index = self.identify_next_waypoint_index()
            self.next_wp_pub.publish(Int32(next_waypoint_index))


    def waypoints_cb(self, lane):
        if hasattr(self, 'waypoints') and self.waypoints != lane.waypoints:
            self.waypoints = lane.waypoints
            self.next_waypoint_index = None


    def traffic_cb(self, traffic_waypoint):
        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not search_range_begin_index waypoint updater node.')
