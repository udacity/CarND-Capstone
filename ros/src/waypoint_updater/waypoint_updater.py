#!/usr/bin/env python

import rospy
from tf import transformations
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import time

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number.
MAX_ACCELERATION = 1.0 # Maximal acceleration [m/s^2]
MAX_DECELERATION = 1.0 # Maximal deceleration [m/s^2]


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add subscriber for /obstacle_waypoint when implemented.
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints            = None # Waypoint list as it is published for the first time
        self.pose                      = None # Current pose
        self.waypoint_index_for_stop   = -1   # Index for waypoint in base waypoints which is closest to stop line of traffic light
        self.closest_waypoint_index    = None # Index for waypoint in base waypoints which is closest to current position
        rospy.spin()

    def pose_cb(self, msg):
        # Callback for /current_pose message.
        # TODO: Implement
        self.pose = msg.pose
        # Check that base waypoints are already received.
        if self.base_waypoints is not None:
            start_time = time.time()
            next_waypoint_index = self.get_next_waypoint_index(self.pose, self.base_waypoints)
            # Create a fixed number of waypoints ahead of the car.
            start = next_waypoint_index
            end   = next_waypoint_index + LOOKAHEAD_WPS
            lookahead_waypoints = self.base_waypoints[start:end]
			# Save velocities of base waypoints. To be fast, this saves only
			# values that are possibly changed.
            original_velocities = []
            for i in range(LOOKAHEAD_WPS):
                original_velocities.append(self.base_waypoints[start+i].twist.twist.linear.x)
            # TODO: Test deceleration.
            #if next_waypoint_index >= self.waypoint_index_for_stop:
            #    self.waypoint_index_for_stop = next_waypoint_index + 400
            # If stop position is valid, decelerate.
            if self.waypoint_index_for_stop >= 0:
                lookahead_waypoints = self.decelerate(lookahead_waypoints, start)
            # Create the Lane object and fill in waypoints.
            lane                 = Lane()
            lane.waypoints       = lookahead_waypoints
            lane.header.frame_id = msg.header.frame_id
            lane.header.stamp    = rospy.Time.now()
            # Publish /final_waypoints topic.
            self.final_waypoints_pub.publish(lane)
			# Restore velocities of base waypoints.
            for i in range(LOOKAHEAD_WPS):
                self.base_waypoints[start+i].twist.twist.linear.x = original_velocities[i]
            end_time = time.time()
            # Log duration.
            #rospy.logwarn('waypoint_updater.py - pose_cb - duration: %5.3f',
            #              (end_time - start_time) * 1000.0)

    def waypoints_cb(self, msg):
        # Callback for /base_waypoints message.
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.waypoint_index_for_stop = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later.
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def calc_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def calc_distance_of_points(self, point_1, point_2):
        # Calculates the distance between two points in 3D.
        return math.sqrt((point_2.x - point_1.x)**2
                       + (point_2.y - point_1.y)**2
                       + (point_2.z - point_1.z)**2)

    def get_closest_waypoint_index(self, pose, waypoints):
        # Gets index of closest waypoint.
        closest_distance = 100000 # large number
        closest_index    = 0
        current_position = pose.position
        # If closest waypoint was already found, reduce search to its neighborhood.
        if self.closest_waypoint_index == None:
            start = 0
            waypoints_reduced = waypoints
        else:
            start = self.closest_waypoint_index - 10
            waypoints_reduced = waypoints[start:start + 100]
        for index, waypoint in enumerate(waypoints_reduced):
            waypoint_position = waypoint.pose.pose.position
            distance = self.calc_distance_of_points(current_position, waypoint_position)
            if closest_distance > distance:
                closest_distance = distance
                closest_index = index + start
        self.closest_waypoint_index = closest_index
        return closest_index

    def get_next_waypoint_index(self, pose, waypoints):
        # Gets next waypoint ahead of the car.
        closest_waypoint_index = self.get_closest_waypoint_index(pose, waypoints)
        waypoint_position = waypoints[closest_waypoint_index].pose.pose.position
        current_position  = pose.position
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = transformations.euler_from_quaternion(quaternion)
        heading = math.atan2((waypoint_position.y - current_position.y),
                             (waypoint_position.x - current_position.x))
        angle = abs(yaw - heading)
        angle = min(2 * math.pi - angle, angle)
        # Check for angles < 90 degree to get a waypoint ahead of the car.
        if angle > math.pi/2:
            closest_waypoint_index += 1
            if closest_waypoint_index == len(waypoints):
                closest_waypoint_index = 0
        return closest_waypoint_index

    def accelerate(self, waypoints, base_offset):
        # TODO: Adjusts the target velocities for the waypoints leading up to top
        # speed in order to accelerate the vehicle.
        for wp in waypoints[stop_index:]:
            wp.twist.twist.linear.x = 0.
        start_wp = self.base_waypoints[start_index]
        for wp in waypoints[:stop_index][::-1]:
            distance = self.calc_distance_of_points(start_wp.pose.pose.position,
                                                    wp.pose.pose.position)
            velocity = math.sqrt(2 * MAX_ACCELERATION * distance)
            if velocity > wp.twist.twist.linear.x - 1.:
                velocity = wp.twist.twist.linear.x
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
        return waypoints

    def decelerate(self, waypoints, base_offset):
        # Adjusts the target velocities for the waypoints leading up to red
        # traffic lights in order to bring the vehicle to a smooth and full stop.
        stop_index = self.waypoint_index_for_stop - base_offset
        if stop_index >= LOOKAHEAD_WPS:
            return waypoints
        for wp in waypoints[stop_index:]:
            wp.twist.twist.linear.x = 0.
        stop_wp = waypoints[stop_index]
        for wp in waypoints[:stop_index][::-1]:
            distance = self.calc_distance_of_points(wp.pose.pose.position,
                                                    stop_wp.pose.pose.position)
            velocity = math.sqrt(2 * MAX_DECELERATION * distance)
            if velocity < 1.:
                velocity = 0.
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
