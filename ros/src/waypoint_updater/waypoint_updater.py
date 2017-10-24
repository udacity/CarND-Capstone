#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
import copy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
STOP_AHEAD_LIGHT = 5 # Number of waypoints the car stops ahead of traffic light


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.decel_limit = 0.8 * abs(rospy.get_param('/dbw_node/decel_limit'))

        ####################### to get traffic light ground truth, needs to be removed afterward ################################
        #self.traffic_light_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_cb)
        ########################################################################################################################

        # find base waypoint index(s) before stop line positions
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.current_velocity = None
        self.waypoints = None
        self.traffic_index = -1
        self.next_waypoint_index = -1
        self.light_points = []

        self.speed_limit = kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

        # modify number of look ahead waypoints based on speed limit
        global LOOKAHEAD_WPS
        LOOKAHEAD_WPS = int(self.speed_limit * 10)

        # let's try to run it a bit faster for more accurate control
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_next_waypoints()
            rate.sleep()

    def construct_stop_index(self):
        # code from tl_dector
        self.light_points = []
        for slpos in self.stop_line_positions:
            dist = [(slpos[0] - wp.pose.pose.position.x) ** 2 + (slpos[1] - wp.pose.pose.position.y) ** 2 for wp in self.waypoints]
            self.light_points.append(np.argmin(dist))

    def traffic_light_cb(self, msg):
        for i in range(len(self.light_points)):
            if self.next_waypoint_index and self.light_points:
                if self.light_points[i] >= self.next_waypoint_index and self.light_points[i] <= self.next_waypoint_index + LOOKAHEAD_WPS:
                    if msg.lights[i].state == 0 or msg.lights[i].state == 1:
                        self.traffic_index = self.light_points[i]
                    else:
                        self.traffic_index = -1

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def vel_cb(self, msg):
        self.current_velocity = msg.twist

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.base_waypoints_sub.unregister()
        self.construct_stop_index()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # self.traffic_index = -1; # to get from traffic_cb
        if msg.data != self.traffic_index:
            self.traffic_index = msg.data
            rospy.logwarn('traffic waypont %d', self.traffic_index)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        n = len(waypoints)
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2):
            dist += dl(waypoints[i%n].pose.pose.position, waypoints[(i+1)%n].pose.pose.position)
        return dist

    def publish_next_waypoints(self):
        if not self.current_pose or not self.waypoints:
            return

        self.next_waypoint_index = self.find_next_waypoint_index()
        if self.next_waypoint_index == -1 or rospy.is_shutdown():
            return

        #rospy.logwarn(
        #    "current position (%.3f, %.3f), next waypoint: %d, traffic waypoint: %d", 
        #    self.current_pose.position.x,
        #    self.current_pose.position.y,
        #    self.next_waypoint_index,
        #    self.traffic_index
        #)

        waypoints = list(self.get_copied_waypoints(self.next_waypoint_index))

        # decelerate if existing red / yellow traffic light
        num_wps = self.num_wps_to_tl()
        if self.traffic_index > - 1 and num_wps < LOOKAHEAD_WPS:
            ahead_idx = max(num_wps-STOP_AHEAD_LIGHT, 0)
            ahead_wp = waypoints[ahead_idx]

            # stop ahead the light
            for i in range(ahead_idx, LOOKAHEAD_WPS):
                self.set_waypoint_velocity(waypoints, i, 0.)

            # gradually decelerate
            for i in range(ahead_idx-1, -1, -1):
                wp = waypoints[i]
                d = self.distance(waypoints, i, i + 1)
                ahead_speed = self.get_waypoint_velocity(ahead_wp)
                speed = math.sqrt(2 * self.decel_limit * d + ahead_speed**2)
                if speed < 1.:
                    speed = 0.
                old_speed = self.get_waypoint_velocity(wp)
                if speed > old_speed:
                    break
                self.set_waypoint_velocity(waypoints, i, speed)
                ahead_wp = wp

        output = Lane()
        output.header.frame_id = '/world'
        output.header.stamp = rospy.Time.now()
        output.waypoints = waypoints

        self.final_waypoints_pub.publish(output)

    def get_copied_waypoints(self, start_idx):
        for i in range(LOOKAHEAD_WPS):
            idx = (start_idx + i + len(self.waypoints)) % len(self.waypoints)
            yield copy.deepcopy(self.waypoints[idx])

    def find_nearest_waypoint(self):
        dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2

        nearest_dist = float('inf')
        nearest_waypoint_index = -1

        for i in range(self.next_waypoint_index, self.next_waypoint_index + len(self.waypoints)):
            idx = (i + len(self.waypoints)) % len(self.waypoints)
            dist = dl(self.current_pose.position, self.waypoints[idx].pose.pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_waypoint_index = idx
            if nearest_dist < 10 and dist > nearest_dist:
                break

        return nearest_waypoint_index, nearest_dist

    def find_next_waypoint_index(self):
        dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2

        nearest_waypoint_index, nearest_dist = self.find_nearest_waypoint()
        if nearest_waypoint_index == -1:
            return -1

        # if we are behind or past the closest waypoint
        wp_pos = self.waypoints[nearest_waypoint_index].pose.pose.position
        pos = copy.deepcopy(self.current_pose.position)
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        )
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        pos.x += x * .1
        pos.y += y * .1
        pos.z += z * .1
        if dl(wp_pos, pos) > nearest_dist:
            nearest_waypoint_index = (nearest_waypoint_index + 1) % len(self.waypoints)

        return nearest_waypoint_index

    def num_wps_to_tl(self):
        if self.traffic_index == -1 or self.next_waypoint_index == -1:
            return -1
        diff = self.traffic_index - self.next_waypoint_index
        if diff < 0:
            diff += len(self.waypoints)
        return diff

def kmph2mps(speed_kmph):
    return (speed_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
