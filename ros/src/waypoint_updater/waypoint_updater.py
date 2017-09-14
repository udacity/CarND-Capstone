#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
import tf
import numpy
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
        self.search_range = 50 # maximum search range to look for waypoints
        self.max_angle_diff = to_rad(30) # max angle difference between consecutive waypoints
        self.previous_wp_pose = []
        self.previous_wp_yaw = []
        self.waypoints = []

        self.current_pose = []

        self.tl_pose = []
        self.tl_state = 'green'

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', ??? , obstacle_cb)   #BUG - there is no obstacle_waypoint

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.loop()
        rospy.spin()

    def get_yaw(self,msg_quat):
        quaternion = [msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        return euler[2]

    def loop(self):
        rospy.loginfo("Waypoint updater loop started")
        while not (self.current_pose):
            pass

        while not (self.waypoints):
            pass

        # debug base_waypoints
        thefile = open('/home/ninopereira/car_test.txt', 'w')
        for wp in self.waypoints:
            thefile.write(str(wp.pose.pose.position.x) + ',' + str(wp.pose.pose.position.y) + '\n')


        updateRate = 5 # update frequency in Hz  Should be 50Hz TBD
        rate = rospy.Rate(updateRate)
        rospy.loginfo("Running with update freq = %s", updateRate)
        while not rospy.is_shutdown():

            rospy.loginfo("Current_pose = %s,%s",self.current_pose.position.x,self.current_pose.position.y)
            # todo: create a safety mechanism
            # if no pose update received stop the car


            # todo: create a message of type Lane
            # lane_msg.header
            # lane_msg.waypoints

            # find the closest waypoint to the current pose
            # use the previous wp yaw to find the next wp with more precision
            ref_distance = self.linear_distance(self.waypoints[0].pose.pose, self.current_pose)
            # rospy.loginfo("ref_distance = %s",ref_distance)
            distance_values = []
            #get near neighbours and corresponding indexes
            for index in range(len(self.waypoints)):
                distance = self.linear_distance(self.waypoints[index].pose.pose, self.current_pose)
                if distance <= self.search_range:
                    distance_values.append([distance, index])
                    #rospy.loginfo("candidate %s, %s", distance, index)

            rospy.loginfo("Testing %s candidates", len(distance_values))
            #rospy.loginfo("candidates", distance_values)
            next_wp_index = -1
            distance_values.sort()

            for i in range(len(distance_values)):
                index = distance_values[i][1]
                wp_yaw = self.get_yaw(self.waypoints[index].pose.pose.orientation)
                angle_diff = abs(((wp_yaw - self.previous_wp_yaw) + math.pi) % (2*math.pi) - math.pi)
                dist_from_previous_wp_to_current_pose = self.linear_distance(self.previous_wp_pose,self.waypoints[index].pose.pose)
                dist_from_previous_wp_to_this_wp = self.linear_distance(self.previous_wp_pose,self.current_pose)
                if (angle_diff < self.max_angle_diff) and (dist_from_previous_wp_to_this_wp >= dist_from_previous_wp_to_current_pose):
                    next_wp_index = index
                    next_wp_yaw = wp_yaw
                    break

            if next_wp_index != -1:
                rospy.loginfo("Publish closest_waypoint with index = %s", next_wp_index)
                self.previous_wp_pose = self.waypoints[next_wp_index].pose.pose
                self.previous_wp_yaw = next_wp_yaw
                # todo: build a new set of waypoints by using only a given number (LOOKAHEAD_WPS)
                # todo: of base waypoints from the current pose onwards

                # todo: we may need to wrap around
                # msg_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]

                # todo: publish the message
                #self.final_waypoints_pub.publish(lane_msg)
            else:
                rospy.logwarn("Failed to find next closest_waypoint")#. Current pose = %s", self.current_pose)

            rate.sleep()

    def pose_cb(self, msg):
        if not self.current_pose:
            self.previous_wp_pose = msg.pose
            self.previous_wp_yaw = self.get_yaw(msg.pose.orientation) # initialize previous wp here
            rospy.loginfo("init yaw = %s", self.previous_wp_yaw)
        self.current_pose = msg.pose
        #rospy.loginfo("cur_position = (%s,%s)", self.current_pose.position.x, self.current_pose.position.y)
        return

    # computes the euclidean distance between 2 poses
    def linear_distance(self, pose1, pose2):
        delta_x = pose1.position.x - pose2.position.x
        delta_y = pose1.position.y - pose2.position.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints #save all the waypoints to internal variable

        return

    def traffic_cb(self, msg):

        # TODO: Callback for /traffic_waypoint message. Implement
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
