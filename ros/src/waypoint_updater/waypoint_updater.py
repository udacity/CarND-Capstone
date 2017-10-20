#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np

import yaml
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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        ####################### to get traffic light ground truth, needs to be removed afterward ################################
        self.traffic_light_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_cb)
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
        self.next_waypoint_index = None
        self.velocity = 0

        self.speed_limit = rospy.get_param("~speed_limit")
        # suggest to reuse this setting for speed limit
        #self.speed_limit = kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

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
        self.velocity = msg.twist

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.base_waypoints_sub.unregister()
        self.construct_stop_index()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # self.traffic_index = -1; # to get from traffic_cb
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

    def publish_next_waypoints(self):
        waypoints = []

        #rospy.logwarn('traffic_index: %d', self.traffic_index)
        if self.current_pose and self.waypoints and self.traffic_index:
            waypoint_begin_index = self.get_next_waypoint_index()
            waypoint_end_index = waypoint_begin_index + LOOKAHEAD_WPS

            #rospy.logwarn('traffic index %d, begin index: %d', self.traffic_index, waypoint_begin_index)

            if waypoint_end_index > len(self.waypoints):
                waypoint_end_index = len(self.waypoints)

            if self.traffic_index == -1 or waypoint_begin_index > self.traffic_index: # case of passing stop line when light turn to red
                for i in range(waypoint_begin_index, waypoint_end_index):
                    self.set_waypoint_velocity(self.waypoints, i, self.speed_limit)
                    waypoints.append(self.waypoints[i])
            else:
                waypoint_end_index = self.traffic_index

                # assign speed target according to location
                num_waypoints = waypoint_end_index - waypoint_begin_index
                multiplier = self.speed_limit / LOOKAHEAD_WPS * 2
                if num_waypoints < LOOKAHEAD_WPS:
                    speed = num_waypoints * multiplier
                else:
                    speed = self.speed_limit

                    #vDot = self.velocity.linear.x / (waypoint_end_index - waypoint_begin_index)
                    #rospy.logwarn('vDot %f', vDot)
                for i in range(waypoint_begin_index, waypoint_end_index):
                    self.set_waypoint_velocity(self.waypoints, i, speed)
                    waypoints.append(self.waypoints[i])

        output = Lane()
        output.waypoints = waypoints

        self.final_waypoints_pub.publish(output)

    def get_nearest_waypoint_index(self):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

        nearest_dist = 99999
        nearest_waypoint_index = 0
        next_waypoint_index = 0

        while next_waypoint_index < len(self.waypoints):
            dist = dl(self.waypoints[next_waypoint_index].pose.pose.position, self.current_pose.position)
            if (dist < nearest_dist):
                nearest_dist = dist
                nearest_waypoint_index = next_waypoint_index
            next_waypoint_index += 1

        return nearest_waypoint_index

    def get_next_waypoint_index(self):
        nearest_waypoint_index = self.get_nearest_waypoint_index()

        if nearest_waypoint_index + 3 < len(self.waypoints):
            ahead_waypoint_index = nearest_waypoint_index + 3
        else:
            ahead_waypoint_index = len(self.waypoints) - 1

        # calculate vectors (current.x - nearest.x, current.y - nearest.y), (current.x - next.x, current.y - next.y)
        # to decide if nearest is the next point or one more
        current_ps = self.current_pose.position
        vec_nearest = np.array([current_ps.x - self.waypoints[nearest_waypoint_index].pose.pose.position.x, current_ps.y - self.waypoints[nearest_waypoint_index].pose.pose.position.y])
        vec_next = np.array([current_ps.x - self.waypoints[ahead_waypoint_index].pose.pose.position.x, current_ps.y - self.waypoints[ahead_waypoint_index].pose.pose.position.y])

        next_waypoint_index = nearest_waypoint_index
        if (vec_nearest.dot(vec_next) < 0):
            next_waypoint_index = next_waypoint_index + 1

        self.next_waypoint_index = next_waypoint_index
        # quaternion = (
        #     self.current_pose.orientation.x,
        #     self.current_pose.orientation.y,
        #     self.current_pose.orientation.z,
        #     self.current_pose.orientation.w,
        # )
        # #euler = tf.transformations.euler_from_quaternion(quaternion)
        #if (euler[2] > (math.pi/4)):
        #    next_waypoint_index += 1

        return next_waypoint_index

def kmph2mps(speed_kmph):
    return (speed_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
