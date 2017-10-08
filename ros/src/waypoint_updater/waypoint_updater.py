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
        self.traffic_stop_waypoint = None

        self.loop_freq = 10 # hertz

        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        self.accel_limit = rospy.get_param('/dbw_node/accel_limit', 1)              

        # Convert kph to meters per sec
        self.velocity = rospy.get_param('/waypoint_loader/velocity', 10) * 0.27778
        self.velocity_min = self.accel_limit * (1./self.loop_freq)

        self.stopped = False
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.loop_freq)
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

                # Current target car velocity in m/s
                vx = self.get_waypoint_velocity(lane.waypoints[0])
                vx2 = vx * vx
       
                if self.traffic_stop_waypoint != None:
                    # Stop light ahead
        
                    # Calculate deceleration needed to stop at traffic stop waypoint
                    stopping_distance = self.distance(self.waypoints, self.next_waypoint_index, self.traffic_stop_waypoint)
                    if stopping_distance > 0:
                        ax = - vx2 / (2. * stopping_distance)
                        if ax < self.decel_limit:
                            ax = self.decel_limit
                    else:
                        ax = self.decel_limit
                        self.stopped = True
                    
                else:
                    # No stop light ahead
                    if vx < self.velocity:
                        ax = self.accel_limit
                        if vx < self.velocity_min:
                            vx = self.velocity_min
                            self.set_waypoint_velocity(self.waypoints, 0, vx)
                            vx2 = vx * vx
                    else:
                        ax = 0
            
                prev_pos = lane.waypoints[0].pose.pose.position
                for wp in lane.waypoints[1:]:
                    next_pos = wp.pose.pose.position
                    dist = self.distance_between_two_points(prev_pos, next_pos)
                    prev_pos = next_pos
                    if ax:
                        vwp2 = vx2 + 2 * ax * dist
                        if vwp2 > 0:
                            vwp = math.sqrt(vwp2)
                        else:
                            vwp = 0
                    else:
                        vwp = vx
                    wp.twist.twist.linear.x = vwp                
                    
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

  
    def distance_between_two_points(self, position1, position2):
        a = position1
        b = position2
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


    # Find waypoint coordinates in Car Coordinate Systems
    # Return dot product of unit yaw vector and car_to_waypoint vector
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
        closest_waypoint_in_car_coordinate_system = ((world_closest_waypoint_x-car_coordinate_x) * math.cos(yaw) + (world_closest_waypoint_y-car_coordinate_y) * math.sin(yaw))
        return closest_waypoint_in_car_coordinate_system

    # Update to the closest waypoint of your current position
    def identify_next_waypoint_index(self):
        # Get the Closest Waypoint to Current Car Position
        closest_waypoint_index = self.find_close_waypoint_in_range()
        # Find Map Coordinates for this WayPoint
        closest_waypoint_in_car_coordinate_system = self.waypoint_in_car_coordinate_system(closest_waypoint_index)
        # If Behind increase the waypoint index 
        if ( closest_waypoint_in_car_coordinate_system < 0. ) :
            closest_waypoint_index += 1
            closest_waypoint_index %= len(self.waypoints)
        if ( closest_waypoint_index == self.next_waypoint_index) :
            closest_waypoint_index += 1
            closest_waypoint_index %= len(self.waypoints)
        self.next_waypoint_index = closest_waypoint_index
        return closest_waypoint_index

    def find_close_waypoint_in_range(self):
        minimum_distance_value = 90000
        closest_waypoint_index = 0
        car_current_position = self.current_pose.pose.position
        nwaypoints = len(self.waypoints)
        if self.next_waypoint_index != None:
            search_idx_list = []
            search_range_begin_index = self.next_waypoint_index - waypoints_search_range
            if search_range_begin_index < 0:
                search_idx_list.extend(range(search_range_begin_index % nwaypoints, nwaypoints))
                search_range_begin_index = 0     
            search_range_end_index = self.next_waypoint_index + waypoints_search_range
            if search_range_end_index > nwaypoints:
                search_idx_list.extend(range(0, search_range_end_index % nwaypoints))
                search_range_end_index = nwaypoints
            search_idx_list.extend(range(search_range_begin_index, search_range_end_index))
        else:
            search_idx_list = range(0, nwaypoints)
        for idx in search_idx_list:
            car_future_position = self.waypoints[idx].pose.pose.position
            dist = self.distance_between_two_points(car_current_position, car_future_position)
            if dist < minimum_distance_value:
                minimum_distance_value = dist
                closest_waypoint_index = idx
        return closest_waypoint_index    


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
        if self.traffic_stop_waypoint is None:
            if self.waypoints and traffic_waypoint.data >= 0 and traffic_waypoint.data < len(self.waypoints):
                # Upcoming stop
                self.traffic_stop_waypoint = traffic_waypoint.data
                rospy.loginfo('Setting traffic stop waypoint to %s', traffic_waypoint.data)
        elif traffic_waypoint.data == 0 and self.stopped:
            # Star moving again
            self.traffic_stop_waypoint = None
            self.stopped = False
            rospy.loginfo('Setting traffic stop waypoint to None')


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
        rospy.logerr('Could not search_range_begin_index waypoint updater node.')
