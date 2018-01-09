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

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.saved_base_waypoints = None
        self.current_pose = None
        self.num_waypoints = None
        self.default_speed = 4.0 # unit: m/s

        rospy.spin()

    def pose_cb(self, msg):
        '''
        Messsage type: geometry_msgs/PoseStamped
        '''
        # TODO: Implement
        self.current_pose = msg

        # Check availability
        if (self.saved_base_waypoints is None) or (self.current_pose is None):
            return

        # find closest point
        cidx = self.find_closest_wp_idx()

        # initial publish msg type
        lane = Lane()

        for i in range(LOOKAHEAD_WPS):
            np = Waypoint()
            cwp = self.saved_base_waypoints[(cidx + i) % self.num_waypoints]
            np.pose.pose.position = cwp.pose.pose.position
            np.twist.twist.linear.x = self.default_speed
            lane.waypoints.append(np)

        self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, msg):
        '''
        Message Type: styx_msgs/Lane
        '''
        # TODO: Implement
        self.saved_base_waypoints = msg.waypoints
        self.num_waypoints = len(msg.waypoints)

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

    def distance_2p(self, p1, p2):
        '''
        input : two waypoint pose
        output: distance between 2 waypoint pose
        '''
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(p1,p2)

    def find_closest_wp_idx(self):
        '''
        search for closest waypoint to the current position
        input: waypoints[], current_pose
        output : waypoint index
        '''
        closest_idx = None
        closest_dist = 1e10
        for idx, wp in enumerate(self.saved_base_waypoints):
            dist = self.distance_2p(wp.pose.pose.position, self.current_pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx

        return closest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
