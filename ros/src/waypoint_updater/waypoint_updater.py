#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
import tf
import math
import copy
import numpy as np

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

LOOKAHEAD_WPS = 50  # Number of waypoints to publish
WP_UPDATE_RATE = 2; # processing frequency
REF_VEL = 4.47
def to_deg(angle):
    return angle*180.0/math.pi


def to_rad(angle):
    return angle*math.pi/180.0


class WaypointUpdater(object):
    def __init__(self):
        # Initiate node
        rospy.init_node('waypoint_updater')

        # Waypoint ahead of vehicle in previous loop iteration
        self.previous_wp_index = -1

        # All waypoints
        self.waypoints = []

        # Current vehicle pose
        self.current_pose = None

        # Traffic light
        self.tf_index = -1  # if -1 then there is no traffic light ahead

        # watchdogs for safety critical handling
        self.current_timestamp = rospy.Time.now()
        self.pose_timestamp = rospy.Time.now()
        self.tf_timestamp = rospy.Time.now()
        # TODO verify if this value is OK
        self.watchdog_limit = 0.5e9 # half second

        # Subscribe to topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', ??? , obstacle_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Loop
        self.loop()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    @staticmethod
    def get_yaw(msg_quat):
        quaternion = [msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        return euler[2]

    def get_next_waypoint_index(self):
        """
            Find the closest waypoint to the current pose
            Use previous waypoint to speed up search
        """

        # Number of waypoints in complete set
        num_waypoints = len(self.waypoints)

        # Determine whether to search all waypoints or only a subset
        if self.previous_wp_index == -1:
            # Search all waypoints when starting
            search_all_waypoints = True
        else:
            # Search all waypoints if vehicle is too far from previous waypoint
            threshold_distance_prev_wp = 2  # Maximum distance for narrow proximity search
            distance_prev_wp = self.linear_distance(self.waypoints[self.previous_wp_index].pose.pose, self.current_pose)
            search_all_waypoints = distance_prev_wp > threshold_distance_prev_wp

        # Define subset of waypoints that will be searched
        if search_all_waypoints:
            search_start = 0
            search_end = num_waypoints
        else:
            search_start = self.previous_wp_index - 5   # Waypoints behind previous waypoint
            search_end = self.previous_wp_index + 50    # Waypoints ahead of previous waypoint

        # Get near neighbours and corresponding indexes
        min_distance = 99999
        closest_wp_index = -1
        for wp in range(search_start, search_end):
            index = wp % num_waypoints  # Handle negative and too large index
            distance = self.linear_distance(self.waypoints[index].pose.pose, self.current_pose)
            if distance <= min_distance:
                closest_wp_index = index
                min_distance = distance

        # Should not be possible
        assert closest_wp_index != -1

        # Step one wp ahead if closest wp is behind ego vehicle
        if self.is_waypoint_ahead(self.waypoints[closest_wp_index].pose.pose):
            next_wp_index = closest_wp_index
        else:
            next_wp_index = (closest_wp_index + 1) % num_waypoints
        self.previous_wp_index = next_wp_index

        return next_wp_index

    def loop(self):
        rospy.loginfo("Waypoint updater loop started")

        # Wait for vehicle pose
        while not self.current_pose:
            pass
        rospy.loginfo("Initial pose = %s,%s", self.current_pose.position.x, self.current_pose.position.y)

        # Wait for waypoints to load
        while not self.waypoints:
            pass
        num_waypoints = len(self.waypoints)
        rospy.loginfo("%s waypoints loaded", num_waypoints)

        # Set update frequency in Hz. Should be 50Hz TBD
        rate = rospy.Rate(WP_UPDATE_RATE)
        rospy.loginfo("Waypoint updater running with update rate = %s Hz", WP_UPDATE_RATE)

        # Loop waypoint publisher
        count_lap = True
        num_laps = 0

        while not rospy.is_shutdown():
            emergency_stop = False
            # handle safety critical failures
            self.current_timestamp = rospy.Time.now()
            if (self.current_timestamp - self.pose_timestamp).nsecs > self.watchdog_limit:
                # stop the car
                rospy.logwarn("Safety hazard: not receiving POSE info for long time. Stopping the vehicle!")
                emergency_stop = True

            if (self.current_timestamp - self.tf_timestamp).nsecs > self.watchdog_limit:
                # stop the car
                rospy.logwarn("Safety hazard: not receiving TRAFFIC_LIGHT info for long time. Stopping the vehicle!")
                emergency_stop = True

            if emergency_stop:
                list_wp_to_pub = copy.deepcopy(self.waypoints[0:LOOKAHEAD_WPS])
                # force linear and angular speed to be zero
                for i in range(len(list_wp_to_pub)):
                    list_wp_to_pub[i].twist.twist.linear.x = 0
                    list_wp_to_pub[i].twist.twist.angular.z = 0
            else:
                # rospy.loginfo("Current_pose = %s,%s",self.current_pose.position.x,self.current_pose.position.y)
                next_wp_index = self.get_next_waypoint_index()
                if next_wp_index != -1:

                    if next_wp_index+LOOKAHEAD_WPS >= num_waypoints:
                        excess = (next_wp_index+LOOKAHEAD_WPS) % num_waypoints
                        # Wrap around
                        list_wp_to_pub = copy.deepcopy(self.waypoints[next_wp_index:]) # create a copy to prevent overwritting the original list
                        list_wp_to_pub.extend(self.waypoints[0:excess])
                        # rospy.loginfo("=====> Wrap around: Publishing %s wp from index = %s (%s+%s)", len(list_wp_to_pub), next_wp_index, len(self.waypoints)-next_wp_index,excess)
                        # rospy.loginfo("=====> Wrap around: Publishing %s wp from index = %s to %s", len(list_wp_to_pub), next_wp_index, excess)
                        if count_lap:
                            num_laps = num_laps + 1
                            rospy.loginfo("num_laps = %s",num_laps)
                            count_lap = False
                    else:
                        list_wp_to_pub = copy.deepcopy(self.waypoints[next_wp_index:next_wp_index+LOOKAHEAD_WPS])
                        # rospy.loginfo("Publishing %s wp from index %s to %s", len(list_wp_to_pub),next_wp_index,next_wp_index+LOOKAHEAD_WPS)
                        count_lap = True

                    # Set reference speed
                    for i in range(len(list_wp_to_pub)):
                        list_wp_to_pub[i].twist.twist.linear.x = REF_VEL

                    if self.tf_index != -1:
                        # handle the case where the tf_wp is wrapped
                        if self.tf_index < next_wp_index:
                            wp_to_end_list = num_waypoints-next_wp_index
                            index_in_wp_list = wp_to_end_list + self.tf_index
                        else:
                            # simple case the tf_index is within the LOOKAHEAD distance
                            index_in_wp_list = self.tf_index - next_wp_index

                        # handle the case where the tf_wp is beyond the LOOKAHEAD wp list
                        if index_in_wp_list < LOOKAHEAD_WPS:
                            # 1st set the velocity of tf_index waypoint and beyond to zero
                            MARGIN = 3
                            for i in range(index_in_wp_list-MARGIN,LOOKAHEAD_WPS):  # set speed to zero a few waypoints before the traffic_lights
                                list_wp_to_pub[i].twist.twist.linear.x = 0.0

                            # for wp before the target tf_index decrease speed gradually until the tf_index
                            ref_speed = 0.0
                            for i in range(index_in_wp_list-MARGIN-1, -1, -1):
                                if list_wp_to_pub[i].twist.twist.linear.x > ref_speed:
                                    list_wp_to_pub[i].twist.twist.linear.x = ref_speed
                                    ref_speed = ref_speed + 0.2
                                    #rospy.loginfo("iteration %s = %s", i, ref_speed)
                                else:
                                    #rospy.loginfo("breaking for loop after %s iterations",i)
                                    break

                    #rospy.loginfo("tf_index = %s \tnext_wp_vel = %s",self.tf_index,list_wp_to_pub[0].twist.twist.linear.x)

                    # Create and publish Lane message
                    lane_msg = Lane()
                    lane_msg.header.frame_id = '/world'
                    lane_msg.header.stamp = rospy.Time(0)
                    lane_msg.waypoints = list_wp_to_pub
                    self.final_waypoints_pub.publish(lane_msg)

                else:
                    rospy.logwarn("Failed to find closest_waypoint.")

            rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.pose_timestamp = rospy.Time.now()
        #rospy.loginfo("cur_position = (%s,%s)", self.current_pose.position.x, self.current_pose.position.y)

    def is_waypoint_ahead(self, wp_pose):
        """
            Determine whether given waypoint is ahead of ego vehicle
        """
        # Waypoint coordinates relative to ego vehicle
        ego_x = self.current_pose.position.x
        ego_y = self.current_pose.position.y
        wp_x = wp_pose.position.x
        wp_y = wp_pose.position.y
        delta_y = wp_y - ego_y
        delta_x = wp_x - ego_x

        # Convert to ego coordinates
        ego_yaw = self.get_yaw(self.current_pose.orientation)
        wp_x_local = delta_x * np.cos(ego_yaw) + delta_y * np.sin(ego_yaw)
        wp_y_local = delta_x * -np.sin(ego_yaw) + delta_y * np.cos(ego_yaw)

        return wp_x_local > 0

    @staticmethod
    def linear_distance(pose1, pose2):
        """
            Computes Euclidean distance between 2 positions
        """
        delta_x = pose1.position.x - pose2.position.x
        delta_y = pose1.position.y - pose2.position.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints  # Save all the waypoints to internal variable

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # rospy.loginfo("traffic_cb msg = %s",msg.data)
        self.tf_index = msg.data
        self.tf_timestamp = rospy.Time.now()
        return

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @staticmethod
    def distance(waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
