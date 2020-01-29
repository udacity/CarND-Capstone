#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.car_state = "Accelerating"
        self.state_changed = True
        self.current_stop_waypoint = -1


        # Used to debug final waypoint speed
        self.myFile = open("/home/student/ros_log/log.txt", "w")

        self.waypoint_debug = True

        if self.waypoint_debug == True:
            str = "      "
            self.myFile.write(str)

            for i in range(1000):
                str = "%5d "%i
                self.myFile.write(str)

            self.myFile.write("\n")
            self.myFile.close()

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                state_changed = False
                break_distance = 15.0
                stop_distance = 10.0
                speed = 30.0

                # Make sure consider the immediate stop waypoint
                stop_waypoint = self.stop_idx
                if stop_waypoint != -1:
                    if self.current_stop_waypoint == -1:
                        self.current_stop_waypoint = stop_waypoint
                    else:
                        if closest_waypoint_idx > self.current_stop_waypoint:
                            self.current_stop_waypoint = stop_waypoint
                else:
                    self.current_stop_waypoint = -1


                # Decide on next action
                if self.current_stop_waypoint == -1:
                    if self.car_state != "Accelerating":
                        self.car_state = "Accelerating"
                        state_changed = True
                else:
                    if self.current_stop_waypoint - closest_waypoint_idx < break_distance + stop_distance and self.car_state == "Accelerating":
                        self.car_state = "Decelerating"
                        state_changed = True

                    #if self.current_stop_waypoint - closest_waypoint_idx < 20 and self.car_state == "Accelerating":
                    #    self.car_state = "Decelerating"
                    #    state_changed = True

                #rospy.logwarn('%s, %d, %d', self.car_state, self.current_stop_waypoint, closest_waypoint_idx)

                if state_changed == True:
                    state_changed = False

                    rospy.logwarn('%s, %d, %d', self.car_state, self.current_stop_waypoint, closest_waypoint_idx)

                    if self.car_state == "Accelerating":
                        for i in range(int(break_distance)):
                            self.base_waypoints.waypoints[closest_waypoint_idx + i].twist.twist.linear.x = speed * i / break_distance

                        for i in range(int(break_distance), int(break_distance + stop_distance)):
                            self.base_waypoints.waypoints[closest_waypoint_idx + i].twist.twist.linear.x = speed

                    if self.car_state == "Decelerating":
                        for i in range(int(break_distance)):
                            self.base_waypoints.waypoints[closest_waypoint_idx + i].twist.twist.linear.x = speed - speed * i / break_distance

                        for i in range(int(break_distance), int(break_distance + stop_distance)):
                            self.base_waypoints.waypoints[closest_waypoint_idx + i].twist.twist.linear.x = 0


                    if self.waypoint_debug == True:
                        self.myFile = open("/home/student/ros_log/log.txt", "a")

                        str = "%5d "%closest_waypoint_idx
                        self.myFile.write(str)

                        for i in range(1000):
                            str = "%5.2f "%self.base_waypoints.waypoints[i].twist.twist.linear.x
                            self.myFile.write(str)

                        self.myFile.write("\n")
                        self.myFile.close()

                self.publish_waypoints(closest_waypoint_idx)

            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_idx = msg.data

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
