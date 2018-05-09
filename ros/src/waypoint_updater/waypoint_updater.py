#!/usr/bin/env python
import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np

'''
This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which
does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status
of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node as well as to
verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

MAX_DECELERATION = 1.0
LOOKAHEAD_WPS = 100  # Number of waypoints we publish
STOP_BUFFER = 3 # Number of waypoints to stop ahead of the stop_line_wp

class WaypointUpdater(object):
    def __init__(self):
        log_level_param = rospy.get_param("/log_level")
        if log_level_param.lower() == 'debug':
            rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        else:
            rospy.init_node('waypoint_updater')

        # ROS publishers
        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.ego = None
        self.current_velocity = 0.0
        self.next_idx = -1
        self.stopline_wp_index = -1

        # ROS subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        # TODO: Add a subscriber for /obstacle_waypoint

        self.publishing_loop()

    def publishing_loop(self):
        '''
        Main loop finding the next LOOKAHEAD_WPS waypoints and publishing
        them in the final_waypoints topic.
        :return: None
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.ego and self.base_waypoints:
                final_waypoints = self.generate_lane()
                self.publish(final_waypoints)
            rate.sleep()

    def pose_cb(self, pose):
        #Get current pose
        self.ego = pose

    def current_velocity_cb(self, msg):
        '''
        Callback providing the current car velocity

        :param msg: Car velocity in meters per second
        :return: None
        '''
        self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        '''
        Get list of waypoints along our lane and build a KD Tree
        :param waypoints:
        '''
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_index = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        #Get desired waypoint velocity
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        #Set a waypoints desired velocity
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        '''
        Calculate distance between waypoints from the list of waypoints
        It is not the direct euclidean distance, it is the distance between the 2 specified
        waypoints going through all the waypoints in-between
        :param waypoints: Set of waypoint
        :param wp1: First waypoint
        :param wp2: Last Waypoint
        :return: Distance
        '''
        dist = 0
        for i in range(wp1, wp2 + 1):
            dist += self.euclidean_dist(waypoints[wp1].pose.pose.position, waypoints[(i)].pose.pose.position)
            wp1 = i
        return dist

    def euclidean_dist(self, pt1, pt2):
        """
        Return the Euclidean distance between two points
        :pt1: geometry_msgs/Point
        :pt2: geometry_msgs/Point
        """
        return math.sqrt((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2 +
                         (pt1.z - pt2.z) ** 2)

    def publish(self, final_waypoints):
        '''
        Publish list of next set of waypoints the car will be to reach
        :return: None
        '''
        final_waypoints.header.frame_id = '/world'
        final_waypoints.header.stamp = rospy.Time.now()
        self.pub.publish(final_waypoints)


    def vector_from_quaternion(self, q):
        #Used to convert to vector coord from quaternion for ego vehicle coord
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def find_next_waypoint(self):
        '''
        Finds the next closest waypoint in front of the vehicle
        :return: Index of the closest waypoint in the waypoint_2d array
        '''
        x = self.ego.pose.position.x
        y = self.ego.pose.position.y
        # This returned index is matched onto the indexes of self.waypoints_2d
        closest_waypoint_index = self.waypoints_tree.query([x,y], 1)[1]

        # Since we want to only return waypoints ahead of the vehicle
        # We test whether the waypoint is ahead or behind of the current
        # position of the car.
        closest_coord = self.waypoints_2d[closest_waypoint_index]
        previous_closest_coord = self.waypoints_2d[closest_waypoint_index-1]

        closest_coord_vec = np.array(closest_coord)
        prev_vec = np.array(previous_closest_coord)
        position_vec = np.array([x, y])

        val = np.dot(closest_coord_vec-prev_vec, position_vec-closest_coord_vec)

        # If the dot product is positive, the car is ahead of the closest waypoint found.
        # Then we take the next one in the list.
        if val > 0:
            closest_waypoint_index = (closest_waypoint_index + 1) % len(self.waypoints_2d)
        return closest_waypoint_index

    def generate_lane(self):
        '''
        Generate a set of waypoints ahead of the car to follow.
        :return: Lane containing the waypoints.
        '''
        lane = Lane()

        closest_index = self.find_next_waypoint()
        furthest_index = closest_index + LOOKAHEAD_WPS


        if (self.stopline_wp_index == -1) or (self.stopline_wp_index >= furthest_index):
            # If we are too far behind a traffic light, no need to modify anything
            lane.waypoints = self.base_waypoints.waypoints[closest_index:furthest_index]
        else:
            stopping_distance = (self.current_velocity ** 2) / (2. * MAX_DECELERATION)
            tl_distance = self.euclidean_dist(self.ego.pose.position, self.base_waypoints.waypoints[self.stopline_wp_index].pose.pose.position) + 2.
            rospy.logdebug(['ego x: ', self.ego.pose.position])
            rospy.logdebug(['ego x: ', self.base_waypoints.waypoints[self.stopline_wp_index].pose.pose.position.x])
            rospy.logdebug(['stopping distance: ', stopping_distance])
            rospy.logdebug(['tl distance: ', tl_distance])
            if tl_distance < stopping_distance:
                # if no room for stopping, no need to slow down
                lane.waypoints = self.base_waypoints.waypoints[closest_index:furthest_index]
            else:
                # We copy the waypoints to avoid modifying the base_waypoints themselves
                # and end up having some issues when we'll loop back through these same points
                base_waypoints = self.base_waypoints.waypoints[closest_index:furthest_index]
                lane.waypoints = self.decelerate_trajectory(base_waypoints, closest_index)

        # Message for the TL_Detector: the index of the first final waypoint among the base_waypoints
        lane.waypoints[0].pose.header.seq = closest_index

        return lane

    def decelerate_trajectory(self, waypoints, closest_index):
        '''
        If we encounter a red light, we modify the given waypoints to
        slow down. To slow down the car, we only need to modify the waypoints
        linear velocity.
        :param waypoints:
        :param closest_index:
        :return: List of waypoints to be published
        '''
        final_wp = []
        for i, base_wp in enumerate(waypoints):
            wp = Waypoint()
            # Copy the base_waypoint position (We don't modify that)
            wp.pose = base_wp.pose

            # Since the stopline_wp_index is a waypoint right on top of the stop
            # line of the traffic light and we want to stop a little bit ahead of that,
            # we withdraw 2 waypoints
            # @todo: Improve that maybe
            stop_index = max(self.stopline_wp_index - closest_index - STOP_BUFFER, 0)
            distance = self.distance(waypoints, i, stop_index)

            # The velocity at each point is a function of the distance to the stop line
            # so that the velocity decreases until reaching 0 when the car will be very
            # close to the stop line
            if distance < 0.5:
                # If the velocity is almost 0, we set it to 0 to have a clear stop
                velocity = 0.
            else:
                velocity = np.sqrt(2. * MAX_DECELERATION * distance)

            # We don't want to increase the velocity, we
            wp.twist.twist.linear.x = min(velocity, base_wp.twist.twist.linear.x)

            final_wp.append(wp)
        return final_wp

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')