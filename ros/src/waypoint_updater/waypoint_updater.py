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
STOP_DISTANCE = 3.00  # Distance in 'm' from TL stop line from which the car starts to stop.
STOP_HYST = 3  # Margin of error for a stopping car.
DECEL_FACTOR = 0.1  # Multiplier to the decel limit.
ACC_FACTOR = 0.5  # Multiplier to the accel limit


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

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
        self.velocity = None
        self.traffic_index = -1  # Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time()  # When traffic light info was received
        self.stop_distance = 0.25
        self.slowdown_rate = 0.5
        # ROS parameters
        self.cruise_speed = None
        self.decel_limit = None
        self.accel_limit = None

        self.run()

    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        """ Store the given map """
        self.base_waypoints = msg.waypoints

    def velocity_cb(self, msg):
        self.velocity = msg.twist

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

    def get_next_waypoints(self, waypoints, start):
        """Return a list of n waypoints ahead of the vehicle"""
        next_waypoints = []
        init_vel = self.velocity.linear.x
        end = start + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        a = self.accel_limit
        for idx in range(start, end):
            dist = self.distance(waypoints, start, idx+1)
            speed = math.sqrt(init_vel**2 + 2 * a * dist)
            if speed > self.cruise_speed:
                speed = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, speed)
            next_waypoints.append(waypoints[idx])
        return next_waypoints

    def kmph_to_mps(self, kmph):
        return 0.278 * kmph

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

        # ROS parameters
        self.cruise_speed = self.kmph_to_mps(rospy.get_param('~/waypoint_loader/velocity', 40.0))
        self.decel_limit = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1)

        while not rospy.is_shutdown():

            if self.base_waypoints is None or self.pose is None or self.frame_id is None or self.velocity is None:
                continue

            # Where in base waypoints list the car is
            car_index = self.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Get subset waypoints ahead
            lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index)

            # Traffic light must be new and near ahead
            is_fresh = rospy.get_time() - self.traffic_time_received < STALE_TIME
            is_close = False

            if (self.traffic_index - car_index) > 0:
                d = self.distance(self.base_waypoints, car_index, self.traffic_index)
                car_wp = self.base_waypoints[car_index]
                if d < car_wp.twist.twist.linear.x ** self.slowdown_rate:
                    is_close = True

            rospy.logdebug('is_fresh: %d; is_close: %d' % (is_fresh, is_close))

            # Set target speeds
            if is_fresh and is_close:
                # Slow down and stop
                rospy.logdebug('constructing stop waypoints ...')
                for i, waypoint in enumerate(lookahead_waypoints):
                    _, waypoint.twist.twist.linear.x = self.get_distance_speed_tuple(car_index + i)

            # Publish
            lane = self.construct_lane_object(lookahead_waypoints)
            rospy.logdebug('Update local path waypoints ...')
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
