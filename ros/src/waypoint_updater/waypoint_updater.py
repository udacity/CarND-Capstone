#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_position = []
        #Node-wide variables
        self.base_waypoints = []
        self.total_waypoints = 0
        # TODO: Add other member variables you need below

        rospy.spin()

    def convert_local(self, waypoint, current_pos):
        #Helper function for converting from global to local coords
        #Grab our waypoint and car position variables
        x_way = waypoint.pose.pose.position.x
        y_way = waypoint.pose.pose.position.y
        x_car = current_pos.pose.position.x
        y_car = current_pos.pose.position.y
        #Convert from Quarternion to Radians
        theta_car = 2*math.acos(current_pos.pose.orientation.w)
        theta_waypoint = 2 * math.acos(waypoint.pose.pose.orientation.w)

        #Perform coordinate localization
        x_shift = x_way - x_car
        y_shift = y_way - y_car
        x = x_shift*math.cos(0 - theta_car) - y_shift*math.sin(0 - theta_car)
        y = x_shift*math.sin(0 - theta_car) + y_shift*math.cos(0 - theta_car)
        return x,y, theta_car, theta_waypoint

    def pose_cb(self, msg):
        #Callback for our current car position recieved
        current_pos = msg
        #x = msg.pose.position.x    #Access x value like this
        #theta = msg.pose.orientation.w # Access w value like this

        #Create list for our published final_waypoints
        final_waypoints_list = []

        #Scan through all base_waypoints
        for i in range(len(self.base_waypoints.waypoints)):
            waypoint = self.base_waypoints.waypoints[i]
            # convert waypoint to local coordinates
            x,y,theta_car, theta_waypoint = self.convert_local(waypoint, current_pos)
            orientation_match = math.cos(theta_waypoint - theta_car)
            # Check if the waypoint is in front of our car, and if the orientation is within +/- pi/4 of our car
            if(x > 0.00 and orientation_match > 0.707 ):
                # Since our waypoints are sequential
                # As soon as we find our first waypoint, we populate the rest of the list with the following waypoints
                for j in range(LOOKAHEAD_WPS):
                    j_mod = i+j%self.total_waypoints
                    final_waypoints_list.append(self.base_waypoints.waypoints[j_mod])
                #Format our message
                msg = Lane()
                msg.waypoints = final_waypoints_list
                self.final_waypoints_pub.publish(msg)
                return
        pass

    def waypoints_cb(self, waypoints):
        #Call back for base_waypoints when our simulator is started
        #first_x = waypoints.waypoints[0].twist.twist.linear.x #Use this format to access waypoint info

        # Set a variable for accessing base_waypoints throughout this node
        self.base_waypoints = waypoints
        # Set a variable for access the total number of base_waypoints throughout this node
        self.total_waypoints = len(self.base_waypoints.waypoints)
        #rospy.logwarn(self.total_waypoints)
        pass


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
