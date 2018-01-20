#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
#        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
#        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None # Store the waypoint list as it is published for the first time
        self.pose           = None # Store current pose
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # For partial waypoint updater, publish just a fixed number of waypoints
        # (LOOKAHEAD_WPS)currently ahead of the vehicle
        
        """
            Steps:
                1) Find closest waypoint
                2) Send the next LOOKAHEAD_WPS waypoints
            """
        
        self.pose = msg.pose
        frame_id  = msg.header.frame_id
        
        if self.base_waypoints is not None:
            closest_waypoint_index = find_closest_waypoint_index(self.pose, self.base_waypoints)
            # To make sure that we don't overshoot the data points
            number_of_points       = min(len(self.base_waypoints), closest_waypoint_index + LOOKAHEAD_WPS)
            lookahead_waypoints    = deepcopy(self.base_waypoints[closest_waypoint_index:number_of_points])
        
        final_waypoints = make_final_waypoints(lookahead_waypoints)
        self.final_waypoints_pub.publish(final_waypoints)
        
        pass

    def find_closest_waypoint_index(self, pose, base_waypoints):
        closest_index = 0
    
        current_position = pose.position
        min_difference   = float('inf')
        
        # Loop through all waypoints and find closest index
        
        for index, waypoint in enumerate(waypoints):
            index_position = waypoint.pose.pose.position
            difference     = get_euclidean_distance(current_position, index_position)
        
            if(difference < min_difference):
                min_difference = difference
                closest_index = index
    
        return closest_index
    
    def get_euclidean_distance(self, waypoint1, waypoint2):
        return math.sqrt((waypoint1.x-waypoint2.x)**2 + (waypoint1.y-waypoint2.y)**2)
    
    def make_final_waypoints(self, frame_id, waypoints):
        # Create the Lane object and fill in waypoints
        lane                 = Lane()
        lane.header.frame_id = frame_id
        lane.waypoints       = waypoints
        lane.header.stamp    = rosepy.Time.now()
        return lane
    
    
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store the base waypoints for use
        self.base_waypoints = waypoints
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
