#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, String, Bool

import math
import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some LOOKAHEAD_WPS distance ahead.

The node receives the waypoint index of the closest waypoint to a stop line belonging to a red light in
front of the car from the node /tl_detector.

Based on that information a velocitry profile is generated that assigns individual velocity values to
the waypoints in front of the vehicle. Those waypoints are published via topic /final_waypoints. This
info is then received by the node /pure_pursuit which creates a velocity command for the dbw_node.
'''

LOOKAHEAD_WPS = 200 # number of waypoints in front of the car
IDEAL_DECEL = 2     # m/s2
MAX_DECEL = 5       # m/s2
IDEAL_ACCEL = 1     # m/s2
MAX_ACCEL = 4       # m/s2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.target_vel_pub = rospy.Publisher('/target_velocity', TwistStamped, queue_size=1)

        # test publisher for debugging
        #self.test = rospy.Publisher('/test', String, queue_size=1)

        # object variables
        self.pose = None
        self.car_position = None
        self.base_waypoints = None
        self.car_velocity = None
        self.speed_limit = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.dbw_status = False

        self.acceleration_status = 0 # 0: nothing / -1: decelerating / 1: accelerating
        self.v1 = 0.
        self.t1 = 0.
        self.a = 0.

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            # if pose (from simulator) and waypoints (from waypoint_loader) are received
            if self.pose and self.waypoint_tree:
                # get list of closest waypoints
                closest_waypoint_idx = self.get_closest_waypoint_id()
                # publish list of closest waypoints ahead of vehicle
                self.publish_waypoints(closest_waypoint_idx)

            rate.sleep()

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # closest waypoint ahead or behind vehicle?
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        # use closest waypoint in front of vehicle
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        vel_twist, final_lane = self.waypoints_and_velocity(closest_idx)
        self.target_vel_pub.publish(vel_twist)
        self.final_waypoints_pub.publish(final_lane)

    def waypoints_and_velocity(self, closest_idx):
        tw = TwistStamped()
        lane = Lane()

        #self.test.publish(str(self.dbw_status))

        if self.dbw_status == False:
            self.acceleration_status = 0
            tw.twist.linear.x = 0
        else:
            farthest_idx = closest_idx + LOOKAHEAD_WPS
            car_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
            lane.waypoints = car_waypoints

            # calculate distance from car to first waypoint
            a = self.car_position
            b = car_waypoints[0].pose.pose.position
            car_wp_distance = math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)

            # no red light in sight
            if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
                tw.twist.linear.x = self.accelerating(IDEAL_ACCEL)
            # red light coming up (in max waypoint range)
            else:
                # create stop index for center of the car relative to the stop line
                stop_idx = max(self.stopline_wp_idx - closest_idx -2, 0)
                # calculate distance from car to stop index
                car_stop_distance = car_wp_distance + self.distance(car_waypoints, 0, stop_idx)

                # car is close to stopline and has low speed (prevent division by zero)
                if car_stop_distance <= 0.1 :
                    # set target velocity to zero
                    tw.twist.linear.x = 0
                else: 
                    # calculate linear deceleration value: a = (v^2) / (2 * s)
                    needed_decel = (self.car_velocity**2) / (2 * car_stop_distance)

                    # physically impossible to stop
                    if needed_decel > MAX_DECEL:
                        # accelerate quickly to target speed
                        tw.twist.linear.x = self.accelerating(MAX_ACCEL)
                    # possible to stop
                    else:
                        # already decelerating
                        if self.acceleration_status == -1:
                            # calculated decel is far different from current path decel --> calculate new velocity profile
                            if abs(needed_decel - self.a) > 5:
                                tw.twist.linear.x = self.new_deceleration(needed_decel)
                            else:
                                # v2 = v1 - a * (t2 - t1)
                                tw.twist.linear.x = max(self.v1 - self.a * (rospy.get_time() - self.t1), 0)
                        # acceleration_status == 0 or 1
                        else:
                            if needed_decel >= IDEAL_DECEL:
                                tw.twist.linear.x = self.new_deceleration(needed_decel)
                                self.acceleration_status = -1
                            # car should accelerate
                            else:
                                tw.twist.linear.x = self.accelerating(IDEAL_ACCEL)  

        tw.header.stamp = rospy.Time.now()

        return tw, lane

    def accelerating(self, acceleration):
        # already accelerating
        if self.acceleration_status == 1:
            # v2 = v1 + a * (t2 - t1)
            return min(self.v1 + self.a * (rospy.get_time() - self.t1), self.speed_limit)
        else:
            self.t1 = rospy.get_time()
            self.v1 = self.car_velocity
            self.a = acceleration
            self.acceleration_status = 1

            return self.car_velocity

    def new_deceleration(self, needed_decel):
        self.t1 = rospy.get_time()
        self.v1 = self.car_velocity
        self.a = needed_decel

        return self.car_velocity

    def dbw_status_cb(self, msg):
        self.dbw_status = msg.data

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        self.car_position = self.pose.pose.position

    def velocity_cb(self, msg):
        self.car_velocity = msg.twist.linear.x        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        self.speed_limit = waypoints.waypoints[0].twist.twist.linear.x
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

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
