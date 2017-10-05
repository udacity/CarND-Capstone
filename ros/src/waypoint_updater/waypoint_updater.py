#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from itertools import chain
from copy import deepcopy
import tf
import tf.transformations as tft
import numpy as np
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
SLOWDOWN_WPS  = 100  # Number of waypoints before traffic light which we reduce speed for
STOP_SPEED    =   0. # Desired speed sent to PID for traffic lights
PUBLISH_PERIOD = rospy.Duration.from_sec(0.5)
CAR_FRAME_NAME = "car"


def position2vec3(pose):
    return (pose.x, pose.y, pose.z)

def quat2vec4(quat):
    return (quat.x, quat.y, quat.z, quat.w)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_waypoints_vis_pub = rospy.Publisher('final_waypoints_vis', Marker, queue_size=1)
        self.last_pose_time = rospy.Time.now()
        self.waypoints_received = False
        self.all_waypoints = []
        self.last_waypoint_idx = 0
        self.red_light_idx = -1
        self.tranform_listener = tf.TransformListener(True, rospy.Duration(10.0))
        # TODO: Add other member variables you need below

        rospy.spin()


    def transformWaypoint(self, matrix, waypoint):
        transformed = Waypoint()

        transformed.pose = self.tranform_listener.transformPose(CAR_FRAME_NAME, waypoint.pose)
        transformed.twist.header = transformed.pose.header
        linear = matrix[:-1, :-1].dot(position2vec3(waypoint.twist.twist.linear))
        angular = matrix[:-1, :-1].dot(position2vec3(waypoint.twist.twist.angular))
        transformed.twist.twist.linear = Vector3(linear[0], linear[1], linear[2])
        transformed.twist.twist.angular = Vector3(angular[0], angular[1], angular[2])

        return transformed

    def pose_cb(self, msg):

        current_time = msg.header.stamp
        br = tf.TransformBroadcaster()
        br.sendTransform(position2vec3(msg.pose.position), quat2vec4(msg.pose.orientation), rospy.Time.now(),
                         CAR_FRAME_NAME, "world")
        if not self.waypoints_received:
            return
        if (current_time > self.last_pose_time + PUBLISH_PERIOD):
            self.last_pose_time = current_time

            concatenated = chain(range(self.last_waypoint_idx, len(self.all_waypoints)),
                                 range(self.last_waypoint_idx))

            x = msg.pose.position.x
            y = msg.pose.position.y

            quaternion = (
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            # direction unit vector
            ux = math.cos(euler[2])
            uy = math.sin(euler[2])

            for idx in concatenated:
                pos = self.all_waypoints[idx].pose.pose
                wx = pos.position.x - x
                wy = pos.position.y - y
                intensity = math.sqrt(wx * wx + wy * wy)
                wx = wx / intensity
                wy = wy / intensity
                # if dot producti is > sqrt(2)/2 which corresponds to 45 degrees
                if (ux * wx + uy * wy) > math.sqrt(2) / 2:

                    try:
                        (position, quaternion) = self.tranform_listener.lookupTransform(CAR_FRAME_NAME, "world", rospy.Time())
                        matrix = tf.transformations.quaternion_matrix(quaternion)

                        next_points = Lane()
                        next_points.header.stamp = rospy.Time.now()
                        #next_points.waypoints = [self.transformWaypoint(matrix, self.all_waypoints[i]) for i in
                        #                        range(idx, (idx + LOOKAHEAD_WPS) % len(self.all_waypoints))]
                        self.update_waypoint_speeds()
                        next_points.waypoints = [self.all_waypoints[i] for i in
                                                range(idx, (idx + LOOKAHEAD_WPS) % len(self.all_waypoints))]
                        self.final_waypoints_pub.publish(next_points)
                        self.last_waypoint_idx = idx
                        rospy.loginfo("Waypoints sent")
                        #generate visualization msg
                        vis_msg = Marker()
                        vis_msg.header.stamp = rospy.Time.now()
                        vis_msg.header.frame_id = CAR_FRAME_NAME
                        vis_msg.id = 0
                        vis_msg.type = vis_msg.LINE_STRIP
                        vis_msg.action = vis_msg.ADD
                        vis_msg.scale.x = 0.2
                        vis_msg.scale.y = 0.2
                        vis_msg.scale.z = 0.2
                        vis_msg.color.a = 1.0
                        vis_msg.color.r = 1.0
                        vis_msg.color.g = 1.0
                        vis_msg.color.b = 0.0
                        for wp in next_points.waypoints:
                            vis_msg.points.append(wp.pose.pose.position)

                        self.final_waypoints_vis_pub.publish(vis_msg)
                        return
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print "LookupException"
                        continue

    def waypoints_cb(self, waypoints):
        if len(waypoints.waypoints) == 0:
            rospy.loginfo("Empty waypoints message received")
            return
        rospy.loginfo("Waypoints received")
        self.all_waypoints = waypoints.waypoints
        self.original_waypoint_speeds = list()
        for idx in range(len(self.all_waypoints)):
            self.all_waypoints[idx].pose.header.frame_id = "world"
            self.all_waypoints[idx].twist.header.frame_id = "world"
            self.original_waypoint_speeds.append(waypoints.waypoints[idx].twist.twist.linear.x)
        self.waypoints_received = True

    def reset_waypoint_speeds(self):
        for idx in range(len(self.all_waypoints)):
            self.all_waypoints[idx].twist.twist.linear.x = self.original_waypoint_speeds[idx]

    def update_waypoint_speeds(self):
        # adjust velocities based on traffic light index
        self.reset_waypoint_speeds();
        if self.red_light_idx != -1:
            #TODO: make this more robust to loops
            start_index = max(0,self.red_light_idx - SLOWDOWN_WPS)
            end_index = self.red_light_idx
            start_speed = self.get_waypoint_velocity(self.all_waypoints[start_index])
            end_speed = STOP_SPEED
            for index in range(start_index,end_index+1):
                anti_weight = (index - start_index) / float(end_index - start_index)
                weight = 1 - anti_weight
                speed = weight*start_speed + anti_weight*end_speed
                print("speed for idx: ", index, " is ",speed)
                self.all_waypoints[index].twist.twist.linear.x = speed

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
