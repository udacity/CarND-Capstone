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
        self.previous_wp_pose = []
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
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        self.loop()


    def get_yaw(self,msg_quat):
        quaternion = [msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        return euler[2]

    def get_next_waypoint(self):
        # find the closest waypoint to the current pose
        # use the previous wp yaw to find the next wp with more precision
        ref_distance = self.linear_distance(self.waypoints[0].pose.pose, self.current_pose)
        # rospy.loginfo("ref_distance = %s",ref_distance)
        distance_values = []

        # todo: speed up search by only look at the next few waypoints since the last one after initialisation

        # get near neighbours and corresponding indexes
        min_distance = 99999
        closest_wp_index = -1
        for index in range(len(self.waypoints)):
            distance = self.linear_distance(self.waypoints[index].pose.pose, self.current_pose)
            if distance <= min_distance:
                closest_wp_index = index
                min_distance = distance

        prev_index = (closest_wp_index - 1) % len(self.waypoints)
        next_index = (closest_wp_index + 1) % len(self.waypoints)
        dist_from_prev_wp_to_current_pose = self.linear_distance(self.waypoints[prev_index].pose.pose, self.current_pose)
        dist_from_next_wp_to_current_pose = self.linear_distance(self.waypoints[next_index].pose.pose, self.current_pose)

        if (dist_from_prev_wp_to_current_pose > dist_from_next_wp_to_current_pose):
            next_wp_index = next_index
        else:
            next_wp_index = closest_wp_index

        return next_wp_index

    def loop(self):
        rospy.loginfo("Waypoint updater loop started")
        init = True
        while not (self.current_pose):
            pass
        rospy.loginfo("Initial pose = %s,%s", self.current_pose.position.x, self.current_pose.position.y)
        while not (self.waypoints):
            pass

        # debug base_waypoints
        #thefile = open('/home/ninopereira/car_test.txt', 'w')
        #for wp in self.waypoints:
        #    thefile.write(str(wp.pose.pose.position.x) + ',' + str(wp.pose.pose.position.y) + '\n')


        updateRate = 3 # update frequency in Hz  Should be 50Hz TBD
        rate = rospy.Rate(updateRate)
        rospy.loginfo("Running with update freq = %s", updateRate)
        while not rospy.is_shutdown():

            #rospy.loginfo("Current_pose = %s,%s",self.current_pose.position.x,self.current_pose.position.y)
            # todo: create a safety mechanism
            # if no pose update received stop the car

            next_wp_index = self.get_next_waypoint()

            if next_wp_index != -1:

                self.previous_wp_pose = self.waypoints[next_wp_index].pose.pose
                # todo: build a new set of waypoints by using only a given number (LOOKAHEAD_WPS)
                # todo: of base waypoints from the current pose onwards
                # todo: we may need to wrap around

                if next_wp_index+LOOKAHEAD_WPS >= len(self.waypoints):
                    excess = (next_wp_index+LOOKAHEAD_WPS)%len(self.waypoints)
                    # wrap around
                    list_wp_to_pub = self.waypoints[next_wp_index:]
                    list_wp_to_pub = list_wp_to_pub + (self.waypoints[0:excess])
                    rospy.loginfo("=====> Wrap around: Publishing %s wp from index = %s (%s+%s)",
                                  len(list_wp_to_pub), next_wp_index, len(self.waypoints)-next_wp_index,excess)
                else:
                    list_wp_to_pub = self.waypoints[next_wp_index:next_wp_index+LOOKAHEAD_WPS]
                    rospy.loginfo("Publishing %s wp from index %s ", len(list_wp_to_pub),next_wp_index)

                # msg_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]

                # create a message of type Lane
                # publish the message
                lane_msg = Lane()
                lane_msg.lane.header.frame_id = '/world'
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
