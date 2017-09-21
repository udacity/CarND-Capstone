#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
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


def to_deg(angle):
    return angle*180.0/math.pi


def to_rad(angle):
    return angle*math.pi/180.0


class WaypointUpdater(object):
    def __init__(self):
        # Waypoint ahead of vehicle in previous loop iteration
        self.previous_wp_pose = []
        self.previous_wp_index = -1

        # All waypoints
        self.waypoints = []

        # Current vehicle pose
        self.current_pose = None

        # Traffic light
        self.tl_pose = []
        self.tl_state = 'green'

        # Initiate node
        rospy.init_node('waypoint_updater')

        # Subscribe to topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', ??? , obstacle_cb)   #BUG - there is no obstacle_waypoint

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

    def get_next_waypoint(self):
        """
            Find the closest waypoint to the current pose
            Use previous waypoint to speed up search
        """

        num_waypoints = len(self.waypoints)

        # Determine subset of waypoints in which to search
        if self.previous_wp_index == -1:
            search_start = 0
            search_end = num_waypoints
        else:
            search_start = self.previous_wp_index - 5
            search_end = self.previous_wp_index + 50

        # Get near neighbours and corresponding indexes
        min_distance = 99999
        closest_wp_index = -1
        for index in range(search_start, search_end):
            distance = self.linear_distance(self.waypoints[index].pose.pose, self.current_pose)
            if distance <= min_distance:
                closest_wp_index = index
                min_distance = distance

        prev_index = (closest_wp_index - 1) % num_waypoints
        next_index = (closest_wp_index + 1) % num_waypoints
        dist_from_prev_wp_to_current_pose = self.linear_distance(self.waypoints[prev_index].pose.pose, self.current_pose)
        dist_from_next_wp_to_current_pose = self.linear_distance(self.waypoints[next_index].pose.pose, self.current_pose)

        # TODO: Improve. Assumes equidistant waypoints.
        if dist_from_prev_wp_to_current_pose > dist_from_next_wp_to_current_pose:
            next_wp_index = next_index
        else:
            next_wp_index = closest_wp_index

        return next_wp_index

    def loop(self):
        rospy.loginfo("Waypoint updater loop started")

        while not self.current_pose:
            pass

        rospy.loginfo("Initial pose = %s,%s", self.current_pose.position.x, self.current_pose.position.y)

        while not self.waypoints:
            pass

        update_rate = 10
        # Update frequency in Hz. Should be 50Hz TBD
        rate = rospy.Rate(update_rate)
        rospy.loginfo("Waypoint updater running with update freq = %s Hz", update_rate)
        while not rospy.is_shutdown():

            #rospy.loginfo("Current_pose = %s,%s",self.current_pose.position.x,self.current_pose.position.y)
            # TODO: Create a safety mechanism if no pose update received stop the car

            next_wp_index = self.get_next_waypoint()

            if next_wp_index != -1:

                self.previous_wp_pose = self.waypoints[next_wp_index].pose.pose
                self.previous_wp_index = next_wp_index

                num_waypoints = len(self.waypoints)
                if next_wp_index+LOOKAHEAD_WPS >= num_waypoints:
                    excess = (next_wp_index+LOOKAHEAD_WPS) % num_waypoints
                    # Wrap around
                    list_wp_to_pub = self.waypoints[next_wp_index:]
                    list_wp_to_pub.extend(self.waypoints[0:excess])
                    #rospy.loginfo("=====> Wrap around: Publishing %s wp from index = %s (%s+%s)", len(list_wp_to_pub), next_wp_index, len(self.waypoints)-next_wp_index,excess)
                else:
                    list_wp_to_pub = self.waypoints[next_wp_index:next_wp_index+LOOKAHEAD_WPS]
                    #rospy.loginfo("Publishing %s wp from index %s ", len(list_wp_to_pub),next_wp_index)

                # msg_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]

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
        if not self.current_pose:
            self.previous_wp_pose = msg.pose
        self.current_pose = msg.pose
        #rospy.loginfo("cur_position = (%s,%s)", self.current_pose.position.x, self.current_pose.position.y)

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
        pass

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
