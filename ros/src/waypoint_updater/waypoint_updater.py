#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import time
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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # not yet implemented, so leaving commented out
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # holds the current vehicle pose from /current_pose topic
        self.current_pose = None

        # holds list of waypoints that will be loaded over /base_waypoints topic
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        print("current pose set!")
        self.publish_waypoints()

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        print("waypoints set!")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        waypoint = msg.data
        rospy.logwarn('waypoint msg data: %d',waypoint)
        if waypoint != -1:
            self.set_waypoint_velocity(self.base_waypoints, waypoint, 0)
            #self.base_waypoints[waypoint].twist.twist.linear.x = 0 # come to stop by here
            # rospy.logwarn('traffic_cb dest_waypoint: %d, current_waypoint: %d', waypoint, self.nearest_waypoint()) #
            for i in range(self.nearest_waypoint(), waypoint-10):
                # rospy.logwarn('updating velocity for waypoint %d', i)
                p = 1. - (i*1.)/(waypoint-10)
                # rospy.logwarn('p: %.3f', p)
                cur_v = self.get_waypoint_velocity(self.base_waypoints[i])
                new_v = cur_v * p
                # rospy.logwarn('   cur_v: %.3f, new_v: %.3f', cur_v, new_v)
                self.set_waypoint_velocity(self.base_waypoints, i, new_v)
            # rospy.logwarn('sent stop')
        else:
            for i in range(self.nearest_waypoint(), self.nearest_waypoint()+100):
                self.set_waypoint_velocity(self.base_waypoints, i, 11.111111)
        self.publish_waypoints()
            #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def position_distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def position_waypoint_distance(self, a, wp):
        return math.sqrt((a.x-wp.pose.pose.position.x)**2 + (a.y-wp.pose.pose.position.y)**2)  #+ (a.z-b.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += position_distance(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint_search(self, cur_wp, waypoints, begin, end):
        if end - begin <= 3:
            nearest_waypoint_index = -1
            nearest_distance = 1e400
            for i in range(begin,end+1):
                dist = self.position_waypoint_distance(cur_wp, waypoints[i])
                if dist<nearest_distance:
                    nearest_distance = dist
                    nearest_waypoint_index = i
            return nearest_waypoint_index, nearest_distance
        low = int((begin * 2 + end) / 3)
        high = int((begin + end * 2) / 3)
        dist_low = self.position_waypoint_distance(cur_wp, waypoints[low])
        dist_high = self.position_waypoint_distance(cur_wp, waypoints[high])
        if dist_low < dist_high:
            return self.closest_waypoint_search(cur_wp, waypoints, begin, high - 1)
        else:
            return self.closest_waypoint_search(cur_wp, waypoints, low + 1, end)

    def nearest_waypoint(self):
        nearest_waypoint_index, nearest_distance = self.closest_waypoint_search(self.current_pose.position,
                                                                                self.base_waypoints,
                                                                                0,
                                                                                len(self.base_waypoints)-1)
        # rospy.logwarn('nearest wp: %d', nearest_waypoint_index)
        # rospy.logwarn('nearest wp dist: %.3f', nearest_distance)
        # rospy.logwarn('nearest search: %.3f'%(1/(time.time()-s)))

        return nearest_waypoint_index

    def publish_waypoints(self):
        nearest_waypoint_index = self.nearest_waypoint()
        closing_waypoint_index = nearest_waypoint_index + LOOKAHEAD_WPS

        waypoints_to_publish = self.base_waypoints[nearest_waypoint_index:closing_waypoint_index]
        #print("publishing {} waypoints".format(len(waypoints_to_publish)))

        self.final_waypoints_pub.publish(Lane(waypoints=waypoints_to_publish))

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
