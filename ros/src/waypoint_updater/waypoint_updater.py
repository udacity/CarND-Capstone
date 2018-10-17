#!/usr/bin/env python2

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math
import yaml

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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = None
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.lane = 1  # 0,1,2
        self.lanetime = 0
        self.prev_pose = None
        self.obstacles = np.array([])

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/vehicle/obstacle_points', PointCloud2, self.obstacle_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        if self.waypoint_tree is not None:
            closest_idx = self.waypoint_tree.query([x, y], 1)[1]
            # check if closest is ahead or behind vehicle
            closest_coord = self.waypoints_2d[closest_idx]
            prev_coord = self.waypoints_2d[closest_idx - 1]
            # equation for hyperplane through closest_coords
            cl_vect = np.array(closest_coord)
            pre_vect = np.array(prev_coord)
            pos_vect = np.array([x, y])
            val = np.dot(cl_vect - pre_vect, pos_vect - cl_vect)
            if val > 0:
                closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            return closest_idx
        return 0

    def get_closest_waypoint_idx1(self, p):
        x = p[0]
        y = p[1]
        if self.waypoint_tree is not None:
            closest_idx = self.waypoint_tree.query([x, y], 1)[1]
            # check if closest is ahead or behind vehicle
            closest_coord = self.waypoints_2d[closest_idx]
            prev_coord = self.waypoints_2d[closest_idx - 1]
            # equation for hyperplane through closest_coords
            cl_vect = np.array(closest_coord)
            pre_vect = np.array(prev_coord)
            pos_vect = np.array([x, y])
            val = np.dot(cl_vect - pre_vect, pos_vect - cl_vect)
            if val > 0:
                closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            return closest_idx
        return 0

    # select the best lane,if the current lane has obstacles,than change to other lane
    # @staticmethod
    # def choose_best_lane(obstacles, lane, s, snum):
    #     item = [100000, 0]
    #     item1 = [item, item]
    #     close_s = [item1]
    #     # calculate the most nearest car before or after the ego car in each lane,get their distance and velocity
    #     for i in range(len(obstacles)):
    #         d = obstacles[i][1]
    #         if (d > 0) and (d < 12):
    #             if d > 8:
    #                 check_lane = 2
    #             elif d < 4:
    #                 check_lane = 0
    #             else:
    #                 check_lane = 1
    #             check_car_s = obstacles[i][0]
    #             pre_dis = (check_car_s - s + snum) % snum
    #             after_dis = (s - check_car_s + snum) % snum
    #             if close_s[check_lane][0][0] > pre_dis:
    #                 close_s[check_lane][0][0] = pre_dis
    #                 close_s[check_lane][0][1] = 0
    #             if close_s[check_lane][1][0] > after_dis:
    #                 close_s[check_lane][1][0] = after_dis
    #                 close_s[check_lane][1][1] = 0
    #     costs = [0, 0, 0]
    #     for j in range(3):
    #         if close_s[j][0][0] <= 50:
    #             # if the distance of the car that is before the ego car in that lane is less then 30 meters,
    #             # don't change
    #             costs[j] = 10000
    #         else:
    #             # if the distance of the car that is after the ego car in that lane is less then 15 meters,
    #             # don't change
    #             if j != lane and close_s[j][1][0] < 15:
    #                 costs[j] = 10000
    #             if costs[j] == 0:
    #                 costs[j] = 1000 * (1 - math.exp(-1 * (1 / (close_s[j][0][0]))))
    #
    #         # decrease the cost of current lane by 1,in order to hold the current lane if it is equal to others.
    #     costs[lane] -= 1
    #     min_cost = costs[0]
    #     min_lane = 0
    #     # select the minimum cost lane
    #     for j in range(3):
    #         if min_cost > costs[j]:
    #             min_cost = costs[j]
    #             min_lane = j
    #     # if change lane from 0 to 2 or from 2 to 0,than look at the cost of lane 1.If the cost of lane 1 is 10000,
    #     # don't change lane.Otherwise,change to lane 1 first.
    #     if abs(min_lane - lane) == 2:
    #         return 1 if costs[1] < 10000 else lane
    #     return min_lane
    #
    # # whether the car is too close to the obstacles,if current lane has obstacles
    # def istooclose(self, closest_idx):
    #     size = len(self.obstacles)
    #     too_close = False
    #     for i in range(size):
    #         d = self.obstacles[i][1]
    #         s = self.obstacles[i][0]
    #         # the vehicle which is changing to this line is also considered as in this lane
    #         if (2 + 4 * self.lane + 2) + 1 > d > (2 + 4 * self.lane - 2) - 1:
    #             if (s > closest_idx) and ((s - closest_idx) < 30):
    #                 too_close = True
    #     return too_close

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    # def publish_waypoints(self):
    #     if self.waypoint_tree:
    #         final_lane = self.generate_lane()
    #         self.final_waypoints_pub.publish(final_lane)

    # def generate_lane(self):
    #     lane = Lane()
    #     closest_idx = self.get_closest_waypoint_idx()
    #     farthest_idx = closest_idx + LOOKAHEAD_WPS
    #     base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
    #
    #     stop_point = -1
    #     if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
    #         lane.waypoints = base_waypoints
    #     elif self.stopline_wp_idx != -1 and self.stopline_wp_idx < farthest_idx:
    #         stop_point = self.stopline_wp_idx
    #
    #         # print('stop:'+str(self.stopline_wp_idx))
    #     if stop_point != -1:
    #         lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx, stop_point)
    #     else:
    #         lane.waypoints = base_waypoints
    #     return lane

    def decelerate_waypoints(self, waypoints, closest_idx, stop_point):
        temp = []
        # rospy.loginfo('%s %s %s',closest_idx,self.stopline_wp_idx,len(waypoints))
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(stop_point - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    @staticmethod
    def distance2line(p0, p1, p2):
        x0 = p0[0]
        y0 = p0[1]
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]
        d = math.sqrt((y2 - y1) * (y2 - y1) + (x1 - x2) * (x1 - x2))
        return (x0 * (y2 - y1) + y0 * (x1 - x2) + y1 * x2 - x1 * y2) / d

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        obstacle_list = []
        if self.waypoint_tree:
            for p in pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')):
                closest_idx = self.waypoint_tree.query([p[0], p[1]], 1)[1]
                prev_idx = (closest_idx - 1) % len(self.waypoints_2d)
                closest_coord = self.waypoints_2d[closest_idx]
                prev_coord = self.waypoints_2d[closest_idx - 1]
                # equation for hyperplane through closest_coords
                cl_vect = np.array(closest_coord)
                pre_vect = np.array(prev_coord)
                pos_vect = np.array([p[0], p[1]])
                val = np.dot(cl_vect - pre_vect, pos_vect - cl_vect)
                if val > 0:
                    prev_idx = closest_idx
                    closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
                distance = self.distance2line(pos_vect, self.waypoints_2d[prev_idx], self.waypoints_2d[closest_idx])
                obstacle_list.append([prev_idx, distance + 6])
            self.obstacles = np.array(obstacle_list)
            # test
            self.obstacles[0][0] = 400
            self.obstacles[0][1] = 6
        #    print(self.obstacles)

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @staticmethod
    def distance(waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
