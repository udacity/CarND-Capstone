#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
PUBLISH_RATE = 20 # Publishing rate (Hz)

max_local_distance = 20.0 # Max waypoint distance we admit for a local minimum (m)

class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.logdebug('Object Waypoints updater created')

        current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #rospy.logdebug('Current position x:', current_pose.Pose.position.x)
        
        base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.logdebug('Base waypoints ', base_waypoints)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.logdebug('Traffic waypoints ', traffic_waypoint)

        obstacle_waypoint = rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)
        #rospy.logdebug('Obstacle waypoints ', obstacle_waypoint)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below	

        rospy.spin()

    # Position of the car
    def pose_cb(self, msg):
        position = msg.pose.position
        # rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z)) line for check the points (is running no need to activate because is writing a lot)
        return msg.pose

    # Waypoints (not complete yet)
    def waypoints_cb(self, msg):
        return msg.waypoints

    # Callback for /traffic_waypoint message. (not complete yet)
    def traffic_cb(self, msg):
        return msg.data

    # Callback for /obstacle_waypoint message. We will implement it later
    def obstacle_cb(self, msg):
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

    # function Closest Waypoints
    def ClosestWaypoint(self, x, y, maps_x, maps_y):

        closestWaypoint = 0

        for i in range (0, len(maps_x)):
            map_x = maps_x[i]
            map_y = maps_y[i]
            #dist = distance(x,y,map_x,map_y) we need to change this calling and adapt to the function distance

            if (dist < closestLen):
               closestLen = dist
               closestWaypoint = i

         return closestWaypoint


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
