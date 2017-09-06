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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # 40-45 hz
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # 40 hz

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # asychronizedly receive data from subscried topics
        self.base_waypoints_msg = None
        self.current_pose_msg = None
        self.publish_rate = 40 # compared to rate of subscribers

        # publish waypoints in a loop with explicit rate
        self.loop()

        rospy.spin()

    def pose_cb(self, msg):
        # update it everytime when received
        self.current_pose_msg = msg

    def waypoints_cb(self, waypoints):
        # 10902 waypoints for the simulation env
        # ideally only need to do it once since the map doesn't change
        # update it anyway since seq of the msg might change
        self.base_waypoints_msg = waypoints

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

    def dist(self, waypoint, current_pose_msg):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        car_x = current_pose_msg.pose.position.x
        car_y = current_pose_msg.pose.position.y
        car_z = current_pose_msg.pose.position.z

        dx = wp_x - car_x
        dy = wp_y - car_y
        dz = wp_z - car_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def ahead_of(self, waypoint, current_pose_msg):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        car_x = current_pose_msg.pose.position.x
        car_y = current_pose_msg.pose.position.y
        car_z = current_pose_msg.pose.position.z

        dx = wp_x - car_x
        dy = wp_y - car_y
        dz = wp_z - car_z

        return dx < 0

    def next_waypoint(self):
        """Get the index of next waypoint ahead of the car, based on
        received current_pose of car and base_waypoints
        """
        waypoints = self.base_waypoints_msg.waypoints
        next_wp_index = 0
        next_wp_dist = self.dist(waypoints[0], self.current_pose_msg)#float('inf')
        for i, wp in enumerate(waypoints):
            if not self.ahead_of(wp, self.current_pose_msg):
                continue
            else:
                d = self.dist(wp, self.current_pose_msg)
                if d < next_wp_dist:
                    next_wp_dist = d
                    next_wp_index = i
        return next_wp_index

    def loop(self):
        """Publish to /final_waypoints with waypoints ahead of car
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # publishing nothing when receiving nothing
            if (self.current_pose_msg is None) or (self.base_waypoints_msg is None):
                continue
            lane = Lane()
            lane.header = self.base_waypoints_msg.header
            lane_start = self.next_waypoint()
            lane.waypoints = self.base_waypoints_msg.waypoints[lane_start:lane_start+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(lane)

            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
