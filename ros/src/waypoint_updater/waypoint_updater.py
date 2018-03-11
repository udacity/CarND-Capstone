#!/usr/bin/env python
"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 20, March 9, 2018
"""


import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import waypoint_utils as utils
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
STALE_TIME = 1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        # other member variables you need below
        self.pose = None
        self.frame_id = None
        self.base_waypoints = None
        self.traffic_index = -1  # Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time()  # When traffic light info was received
        self.stop_distance = 0.25
        self.slowdown_rate = 0.5

        self.run()

    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        """ Store the given map """
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. Implement
        self.traffic_index = msg.data
        self.traffic_time_received = rospy.get_time()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_distance_speed_tuple(self, index):
        """
        Return tuple of distance from traffic light
        and target speed for slowing down
        """
        d = self.distance(self.base_waypoints, index, self.traffic_index)
        car_wp = self.base_waypoints[index]
        car_speed = car_wp.twist.twist.linear.x
        speed = 0.0

        if d > self.stop_distance:
            speed = (d - self.stop_distance) * (car_speed ** (1-self.slowdown_rate))

        if speed < 1.0:
            speed = 0.0
        return d, speed

    def run(self):
        """
        Continuously publish local path waypoints with target velocities
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue

            # Where in base waypoints list the car is
            car_index = utils.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Get subset waypoints ahead
            lookahead_waypoints = utils.get_next_waypoints(self.base_waypoints, car_index, LOOKAHEAD_WPS)

            # Traffic light must be new and near ahead
            is_fresh = rospy.get_time() - self.traffic_time_received < STALE_TIME
            is_close = False

            if (self.traffic_index - car_index) > 0:
                d = utils.distance(self.base_waypoints, car_index, self.traffic_index)
                car_wp = self.base_waypoints[car_index]
                if d < car_wp.twist.twist.linear.x ** self.slowdown_rate:
                    is_close = True

            # Set target speeds
            if is_fresh and is_close:
                # Slow down and stop
                for i, waypoint in enumerate(lookahead_waypoints):
                    _, waypoint.twist.twist.linear.x = self.get_distance_speed_tuple(car_index + i)

            # Publish
            lane = utils.construct_lane_object(self.frame_id, lookahead_waypoints)
            # rospy.loginfo('Update local path waypoints ...')
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
