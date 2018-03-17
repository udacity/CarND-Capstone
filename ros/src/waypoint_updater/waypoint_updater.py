#!/usr/bin/env python
"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 20, March 9, 2018
"""


import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
from copy import deepcopy
from enum import Enum

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
SAFETY_BUFFER = 0.5


class State(Enum):
    ACCELERATION = 1
    DECELERATION = 2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        # other member variables you need below
        self.pose = None
        self.frame_id = None
        self.base_waypoints = None
        self.num_waypoints = None
        self.final_waypoints = None
        self.current_state = State.DECELERATION
        self.state_changed = True
        self.velocity = None
        self.traffic_index = -1  # Where in base waypoints list the traffic light is

        # ROS parameters
        self.cruise_speed = None
        self.decel_limit_max = None
        self.decel_limit_min = None
        self.accel_limit = None

        self.run()

    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        """ Store the given map """
        self.base_waypoints = msg.waypoints
        self.num_waypoints = len(msg.waypoints)

    def velocity_cb(self, msg):
        self.velocity = msg.twist

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. Implement
        self.traffic_index = msg.data
        if self.traffic_index != -1 and self.num_waypoints is not None:
            self.traffic_index = (self.traffic_index - 5 + self.num_waypoints) % self.num_waypoints

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def kmph_to_mps(self, kmph):
        return 0.278 * kmph

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def construct_lane_object(self, waypoints):
        """ Lane object contains the list of final waypoints ahead with velocity"""
        lane = Lane()
        lane.header.frame_id = self.frame_id
        lane.waypoints = waypoints
        lane.header.stamp = rospy.Time.now()
        return lane

    def get_euler(self, pose):
        """ Returns the roll, pitch yaw angles from a Quaternion \
        Args:
            pose: geometry_msgs/Pose.msg

        Returns:
            roll (float), pitch (float), yaw (float)
        """
        return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                         pose.orientation.y,
                                                         pose.orientation.z,
                                                         pose.orientation.w])

    def is_waypoint_behind(self, pose, waypoint):
        """Take a waypoint and a pose , do a coordinate system transformation
        setting the origin at the position of the pose object and as x-axis
        the orientation of the z-axis of the pose

        Args:
            pose (object) : A pose object
            waypoints (object) : A waypoint object

        Returns:
            bool : True if the waypoint is behind the car else False

        """
        _, _, yaw = self.get_euler(pose)
        originX = pose.position.x
        originY = pose.position.y

        shift_x = waypoint.pose.pose.position.x - originX
        shift_y = waypoint.pose.pose.position.y - originY

        x = shift_x * math.cos(0 - yaw) - shift_y * math.sin(0 - yaw)

        if x > 0:
            return False
        return True

    def get_closest_waypoint_index(self, pose, waypoints):
        """
        pose: geometry_msg.msgs.Pose instance
        waypoints: list of styx_msgs.msg.Waypoint instances
        returns index of the closest waypoint in the list waypoints
        """
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

        best_gap = float('inf')
        best_index = 0
        my_position = pose.position

        for i, waypoint in enumerate(waypoints):

            other_position = waypoint.pose.pose.position
            gap = dl(my_position, other_position)

            if gap < best_gap:
                best_index, best_gap = i, gap

        is_behind = self.is_waypoint_behind(pose, waypoints[best_index])
        if is_behind:
            best_index += 1
        return best_index

    def accelerate_waypoints(self, waypoints, start):
        """Return a list of n waypoints ahead of the vehicle"""
        next_waypoints = []
        init_vel = self.velocity.linear.x
        end = start + LOOKAHEAD_WPS
        accel = 0.5 * self.accel_limit
        for idx in range(start, end):
            dist = self.distance(waypoints, start, (idx+1)%len(waypoints))
            speed = math.sqrt(init_vel**2 + 2 * accel * dist)
            if speed > self.cruise_speed:
                speed = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, speed)
            next_waypoints.append(deepcopy(waypoints[idx]))
        return next_waypoints

    def decelerate_waypoints(self, waypoints, start):
        next_waypoints = []
        init_vel = self.velocity.linear.x
        end = start + LOOKAHEAD_WPS
        dist_to_tl = self.distance(waypoints, start, self.traffic_index)
        decel = init_vel ** 2 / (2 * dist_to_tl)
        # if decel > self.decel_limit_max:
        #     decel = self.decel_limit_max
        for idx in range(start, end):
            dist = self.distance(waypoints, start, (idx+1)%len(waypoints))
            if idx < self.traffic_index:
                vel2 = init_vel**2 - 2 * decel * dist
                if vel2 < 1.0:
                   vel2 = 0.0
                speed = math.sqrt(vel2)
            else:
                speed = 0
            self.set_waypoint_velocity(waypoints, idx, speed)
            next_waypoints.append(deepcopy(waypoints[idx]))
        return next_waypoints

    def continue_with_current_state(self, waypoints, start):
        next_waypoints = []
        j = 0
        while j < len(self.final_waypoints):
            if self.final_waypoints[j].pose.pose.position == waypoints[start].pose.pose.position:
                break
            j += 1
        for i in range(j, len(self.final_waypoints)):
            current_waypoint = deepcopy(waypoints[(start + i - j) % len(waypoints)])
            current_waypoint.twist.twist.linear.x = self.final_waypoints[i].twist.twist.linear.x
            next_waypoints.append(current_waypoint)
        for i in range(len(next_waypoints), LOOKAHEAD_WPS):
            current_waypoint = deepcopy(waypoints[(start + i) % len(waypoints)])
            current_waypoint.twist.twist.linear.x = self.cruise_speed
            next_waypoints.append(current_waypoint)
        return  next_waypoints

    def get_next_waypoints(self, waypoints, car_index):
        """Return a list of n waypoints ahead of the vehicle"""

        # Handle state changes
        if self.current_state == State.ACCELERATION:
            if self.traffic_index != -1:
                brake_distance = self.distance(waypoints, car_index, self.traffic_index) - SAFETY_BUFFER

                min_brake_distance = 0.5 * self.velocity.linear.x ** 2 / self.decel_limit_max
                max_brake_distance = 0.5 * self.velocity.linear.x ** 2 / self.decel_limit_min
                if max_brake_distance >= brake_distance >= min_brake_distance:
                    self.current_state = State.DECELERATION
                    self.state_changed = True

        elif self.current_state == State.DECELERATION:
            if self.traffic_index == -1:
                self.current_state = State.ACCELERATION
                self.state_changed = True

        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        # Handle states
        if self.current_state == State.ACCELERATION and self.state_changed:
            next_waypoints = self.accelerate_waypoints(waypoints, car_index)
        elif self.current_state == State.ACCELERATION and not self.state_changed:
            next_waypoints = self.continue_with_current_state(waypoints, car_index)
        elif self.current_state == State.DECELERATION and self.state_changed:
            next_waypoints = self.decelerate_waypoints(waypoints, car_index)
        elif self.current_state == State.DECELERATION and not self.state_changed:
            next_waypoints = self.continue_with_current_state(waypoints, car_index)
        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        self.state_changed = False

        return next_waypoints

    def run(self):
        """
        Continuously publish local path waypoints with target velocities
        """
        rate = rospy.Rate(5)

        # ROS parameters
        self.cruise_speed = self.kmph_to_mps(rospy.get_param('~/waypoint_loader/velocity', 40.0))
        self.decel_limit_max = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.decel_limit_min = min(1.0, self.decel_limit_max / 2.0)
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1)

        while not rospy.is_shutdown():

            if self.base_waypoints is None or self.pose is None or self.frame_id is None or self.velocity is None:
                continue

            # Where in base waypoints list the car is
            car_index = self.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # generate new path
            lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index)

            # Publish
            lane = self.construct_lane_object(lookahead_waypoints)
            self.final_waypoints = deepcopy(lane.waypoints)
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
