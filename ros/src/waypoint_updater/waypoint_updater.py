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


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.currpose = None
        self.curr_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        self.currpose = msg.pose.position
        pass

    def waypoints_cb(self, lanemsg):
        if self.currpose == None:
           return

        self.curr_waypoints = lanemsg.waypoints
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        mindist = 1000000
        start_idx = 0

        for i in range(len(self.curr_waypoints)):
            a = self.curr_waypoints[i].pose.pose.position
            b = self.currpose
            dist = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            if dist < mindist and (a.x > b.x):
                start_idx = i
                mindist = dist
            if start_idx == (len(self.curr_waypoints) - 1):
                start_idx = 0
        
        idx = 0
        reset = 0
        # Collecting the waypoints ahead of the car. 
        # Wrap around when we reach the end.
        for i in range(LOOKAHEAD_WPS):
            if (i + start_idx > (len(self.curr_waypoints) - 1)) and (not reset):
                start_idx = 0
                idx = 0
                reset = 1
            elif reset:
                idx += 1
            else:
                idx = i + start_idx
            lane.waypoints.append(self.curr_waypoints[idx])
         
        #rospy.logerr('Start idx: %s', start_idx)
        #rospy.logerr('Length lane: %s', len(lane.waypoints))
        self.final_waypoints_pub.publish(lane)
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
