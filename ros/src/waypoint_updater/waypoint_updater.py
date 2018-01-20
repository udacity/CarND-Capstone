#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

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

MAX_JERK  = 10 # m/s2
MAX_ACC   = 10 # m/s3

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # XXX - there is no /obstacle_waypoint yet
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.base_lane_len = None
        self.waypoint_idx = 0 # ever increasing number
        self.last_distance = sys.float_info.max
        self.nearest_tl_idx = None
        self.max_velocity = None
        self.current_velocity = None
        rospy.spin()

    def pose_distance(self, waypoint): 
        a=waypoint.pose.pose.position
        b=self.pose.pose.position
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def map_idx(self, idx):
        return (idx % self.base_lane_len)
    
    def set_closest_mapindex(self):
        distance = sys.float_info.max
        lidx = self.waypoint_idx
        for index in range(0, self.base_lane_len):
            lidx = self.waypoint_idx + index
            tindex = self.map_idx(lidx)
            wayp = self.base_lane.waypoints[tindex]
            newdist = self.pose_distance(wayp)
            if distance > newdist:
                distance = newdist
                self.waypoint_idx = lidx 
            else:
                break
        # check if leastd_index or leastd_index-1
        rospy.loginfo(
            "waypoint_updater: i: {} d: {}"
            .format(self.map_idx(self.waypoint_idx), distance)
        )

    def calculate_yaw(self, new_pose):
        orn = new_pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([orn.x,orn.y,orn.z,orn.w])
        return euler[2]  
    
    def calculate_velocity(self, pose_a, pose_b):
        timedur = pose_b.header.stamp - pose_a.header.stamp
        dist = self.distanceBetweenPoints(pose_a.pose.position, pose_b.pose.position)
        return (float(dist)/timedur.to_sec())

    def pose_cb(self, msg):
        if self.pose is not None and msg is not None:
            self.current_yaw = self.calculate_yaw(msg)
            self.current_velocity = self.calculate_velocity(self.pose, msg)
            rospy.logdebug(
                "waypoint_updater: current velocity of car {} yaw {}"
                .format(self.current_velocity, self.current_yaw)
            )

        self.pose = msg
        if self.base_lane is None:
            return
        self.set_closest_mapindex()
        # create lane object
        lane_op = Lane()
        # copy waypoints ahead of pose and pass it to lane_op
        tmap_idx = self.map_idx(self.waypoint_idx)
        if tmap_idx + LOOKAHEAD_WPS < self.base_lane_len:
            lane_op.waypoints = self.base_lane.waypoints[tmap_idx:tmap_idx+LOOKAHEAD_WPS]
        else:
            start = tmap_idx
            end = self.map_idx(tmap_idx + LOOKAHEAD_WPS)
            lane_op.waypoints = self.base_lane.waypoints[start:] + self.base_lane.waypoints[0:end]
        
        # waypoints are available , recalculate velocity for each using nearest_tl_idx
        if self.nearest_tl_idx is not None:
            # rospy.logerr("Nearest traffic light index {}".format(self.nearest_tl_idx))
            if self.nearest_tl_idx > tmap_idx:
                rel_idx = self.nearest_tl_idx - tmap_idx
                if rel_idx < LOOKAHEAD_WPS:
                    # rospy.logerr("stop detected , slow down till {} and then accelerate".format(rel_idx))
                    lane_op.waypoints[:rel_idx] = self.decelerateToZero(lane_op.waypoints[:rel_idx])
                    lane_op.waypoints[rel_idx:] = self.accelerateToMax(lane_op.waypoints[rel_idx:])
                else:
                    rospy.logdebug("waypoint_updater: traffic light out of range")
                    # no need to consider yet
            else:
                rel_idx = self.nearest_tl_idx + self.base_lane_len - tmap_idx
                if rel_idx < LOOKAHEAD_WPS:
                    # rospy.logerr("stop detected , slow down till {} and then accelerate".format(rel_idx))
                    lane_op.waypoints[:rel_idx] = self.decelerateToZero(lane_op.waypoints[:rel_idx])
                    lane_op.waypoints[rel_idx:] = self.accelerateToMax(lane_op.waypoints[rel_idx:])
                else:
                    rospy.logdebug("waypoint_updater: traffic light out of range")
                    # no need to consider it yet
        # publish lane object
        rospy.logdebug(
            "waypoint_updater: velocities {}"
            .format([self.get_waypoint_velocity(wp) for wp in lane_op.waypoints])
        )

        self.final_waypoints_pub.publish(lane_op)

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        self.base_lane_len = len(self.base_lane.waypoints)

    def traffic_cb(self, msg):
        self.nearest_tl_idx = msg.data
        rospy.logdebug(
            "waypoint_updater: Received nearest traffic light index {}"
            .format(self.nearest_tl_idx)
        )

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    # 2ad = vf2 - vi2 (special case where vi = 0)
    def accelerateToMax(self, waypoints):
        if self.max_velocity is None:
            self.max_velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        first = waypoints[0]
        first.twist.twist.linear.x = 0
        for wp in waypoints[1:]:
            dist = self.distanceBetweenPoints(wp.pose.pose.position, first.pose.pose.position)
            vel = math.sqrt(2 * MAX_ACC * dist)
            if vel > self.max_velocity:
                vel = self.max_velocity
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    # 2ad = vi2 (special case where vf = 0)
    def decelerateToZero(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        
        for wp in waypoints[:-1][::-1]:
            dist = self.distanceBetweenPoints(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_ACC * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def distanceBetweenPoints(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

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
