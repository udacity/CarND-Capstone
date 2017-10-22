#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf
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

CONVERSION_FACTOR = 0.447039 # Factor for converting MPH (miles per hour) in MPS (meters per second)
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. Number to adapt
MAX_SPEED = 20 * CONVERSION_FACTOR #Speed limit (in MPS)

MAX_DECEL = 0.5
STOP_DIST = 5.0

class WaypointUpdater(object):
    def __init__(self):
        # initialize the node waypoint_updater
        rospy.init_node('waypoint_updater')

        # subscribe to the topic /current_pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        # subscribe to the topic /base_waypoints
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_pose = None
        self.waypoints = None
        self.waypoint_on_red_light = -1
        self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        #set pose to msg.pose
        self.current_pose = msg.pose
        self.publish()

    def waypoints_cb(self, lane):
        # TODO: Implement
        # set waypoints
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            self.publish()

    def traffic_cb(self, msg):
        self.waypoint_on_red_light = msg.data
        rospy.loginfo("Detected light: " + str(msg.data))
        if self.waypoint_on_red_light > -1:
            self.publish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2 + 1):
            dist += self.distance_euclid(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_euclid(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x * x + y * y + z * z)

    # funtion to compute the closest waypoint to our current position
    def closest_waypoint(self, pose):
        # simply take the code from the path planning module and re-implement it here
        closest_len = 100000
        closest_waypoint = 0
        for index, waypoint in enumerate(self.waypoints):
            dist = self.distance_euclid(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint = index

        return closest_waypoint

    # funtion to compute the next waypoint to our current position
    # the next waypoint is the closest point in front of our car
    def next_waypoint(self, pose):
        waypoints = self.waypoints
        # compute the closest waypoint to our position
        closest_waypoint = self.closest_waypoint(pose)
        # determine map_x and map_y
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y
        # compute the heading
        heading = math.atan2(map_y-pose.position.y,map_x-pose.position.x)
        # compute yaw using transformations euler from quaternion
        orientations = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        (_,_,yaw) = tf.transformations.euler_from_quaternion(orientations)
        # compute the angle difference between yaw and heading
        angle = math.fabs(yaw - heading)
        if angle > (math.pi/4):
            closest_waypoint = (closest_waypoint + 1) % len(waypoints)

        return closest_waypoint

    def slow_down(self, waypoints, redlight_index):
        if len(waypoints) < 1:
            return []

        last = waypoints[redlight_index]

        last.twist.twist.linear.x = 0.
        # start from the waypoint before last and go backwards
        for index, wp in enumerate(waypoints):

            if index > redlight_index:
                vel = 0
            else:
                dist = self.distance_euclid(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0, dist - STOP_DIST)
                vel = math.sqrt(2 * MAX_DECEL * dist)
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def publish(self):

        if self.current_pose is not None:
            next_waypoint_index = self.next_waypoint(self.current_pose)
            lookahead_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index + LOOKAHEAD_WPS]

            if self.waypoint_on_red_light is None or self.waypoint_on_red_light < 0:

                # set the velocity for lookahead waypoints
                for i in range(len(lookahead_waypoints) - 1):
                    # convert 10 miles per hour to meters per sec
                    self.set_waypoint_velocity(lookahead_waypoints, i, MAX_SPEED)

            else:
                redlight_lookahead_index = max(0, self.waypoint_on_red_light - next_waypoint_index)
                lookahead_waypoints = self.slow_down(lookahead_waypoints, redlight_lookahead_index)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = lookahead_waypoints

            self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')