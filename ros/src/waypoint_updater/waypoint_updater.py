#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
REFRESH_RATE = 30 # Refresh final waypoints at a rate of 30 Hz


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        self.base_waypoint_count = None

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(REFRESH_RATE)
        while not rospy.is_shutdown():
            if self.current_pose and self.base_waypoints:
                closest_waypoint_idx = self.find_closest_waypoint_idx()
                final_waypoints = self.construct_final_waypoints(closest_waypoint_idx)
                self.final_waypoints_pub.publish(final_waypoints)
            rate.sleep()
            
    def find_closest_waypoint_idx(self):
        # find the index of the base waypoint which is closest to the vehicle
        x, y = (self.current_pose.position.x, self.current_pose.position.y)
        closest_waypoint_idx = None
        closest_distance = None
        for i in range(self.base_waypoint_count):
            (wx, wy) = self.unzip_waypoint_coords(self.base_waypoints.waypoints[i])
            distance = math.sqrt(pow((x-wx), 2) + pow((y-wy), 2))
            if distance < closest_distance or not closest_distance:
                closest_distance = distance
                closest_waypoint_idx = i
        
        # verify that the vehicle is behind the detected nearest waypoint
        vehicle_coords = (x,y)
        waypoint_coords = self.unzip_waypoint_coords(self.base_waypoints.waypoints[closest_waypoint_idx])
        previous_waypoint_coords = self.unzip_waypoint_coords(self.base_waypoints.waypoints[closest_waypoint_idx-1])
        if not self.is_behind(vehicle_coords, waypoint_coords, previous_waypoint_coords):
            # detected waypoint is behind vehicle, therefore select next waypoint
            closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.base_waypoints.waypoints)
            
        return closest_waypoint_idx
        
    def unzip_waypoint_coords(self, waypoint):
        return (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)
        
    def is_behind(self, ref_point_coords, waypoint_coords, previous_waypoint_coords):
        # is reference point behind waypoint?
        (x, y) = ref_point_coords
        (wx, wy) = waypoint_coords
        (wprev_x, wprev_y) = previous_waypoint_coords
        
        # evaluate dot product defining plane perpendicular to road
        (v1x, v1y) = (wx - wprev_x, wy - wprev_y)
        (v2x, v2y) = (x - wx, y - wy)
        normal = v1x * v2x + v1y * v2y
        return normal < 0
            
    def construct_final_waypoints(self, start_idx):
        end_idx = start_idx + LOOKAHEAD_WPS
        final_waypoints = Lane()
        final_waypoints.header = self.base_waypoints.header
        final_waypoints.waypoints = self.base_waypoints.waypoints[start_idx:end_idx]
        return final_waypoints
        
    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        if not self.base_waypoints:
          self.base_waypoints = waypoints
          self.base_waypoint_count = len(waypoints.waypoints)

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
