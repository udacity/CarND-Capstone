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
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.last_wp_id = None
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.send_next_waypoints()

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            self.send_next_waypoints()

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

    def send_next_waypoints(self):
        if self.waypoints is None or self.current_velocity is None or self.current_pose is None:
            return
        speed = self.current_velocity.twist.linear.x
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        min_dist = sys.maxsize #64 bit is 2**63 - 1
        min_loc = None

        start_index = 0
        end_index = len(self.waypoints)

        for i in range(start_index, end_index):
            waypoint = self.waypoints[i]
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            # distance calulated with distance function
            dist = math.sqrt((car_x - wp_x)**2 + (car_y - wp_y)**2)

            # need to make sure the waypoint is in front of the car
            print "car_x", car_x
            print "car_y", car_y
            print "wp_x", wp_x
            print "wp_y", wp_y
            if dist < min_dist:
                min_dist = dist
                min_loc = i

        closest_wp = self.waypoints[min_loc]
        closest_wp_pos = closest_wp.pose.pose.position
        self.last_wp_id = min_loc

        # Now that we have the shortest distance, get the next LOOKAHEAD_WPS waypoints.
        # This next line ensures that we loop around to the start of the list if we've hit the end.
        next_wps = list(islice(cycle(self.waypoints), min_loc, min_loc + LOOKAHEAD_WPS - 1))

        lane = Lane()
        lane.waypoints = next_wps
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
