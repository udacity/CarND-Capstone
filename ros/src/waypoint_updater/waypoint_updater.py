#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import waypoint_lib.helper as helper

import math
import tf

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
ONE_MPH = 0.44704 # mph to mps
TARGET_SPEED = 8.0 * ONE_MPH
MAX_DECEL = 5.0 # m/s/s

dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.red_light_wp = -1

        self.slowing_down = False

        # For Debugging
        self.cnt = 0

        rospy.spin()


    def pose_cb(self, pose):
        # TODO: Implement
        # rospy.loginfo('pose cb!!!!')
        # rospy.loginfo('Current pose = {}'.format(msg))
        if self.waypoints is None:
            rospy.loginfo('None waypoints')
            return



        # dists = [self.dist_pose_waypoint(pose, wp) for wp in self.waypoints]
        # closest_waypoint = dists.index(min(dists))

        waypoints_num = len(self.waypoints)

        wp_next = helper.next_waypoint_idx(pose, self.waypoints)

        final_waypoints = []

        target_speed = TARGET_SPEED

        # Distance to red light in waypoints
        wps_to_light = (self.red_light_wp - wp_next + waypoints_num) % waypoints_num
        # if wps_to_light < 0:
        #     wps_to_light += waypoints_num

        # Final waypoint of our path
        la_wp = (wp_next + LOOKAHEAD_WPS) % waypoints_num

        # Distance to stop line
        rl_stop_line_nearest = 20

        # Distance to red_light where to stop (in waypoints)
        # TODO: Make the whole stopping in front of a stop line smooth
        # need to adjust distances where to stop based on current velocity and
        # find out correct params for this. [Pavlo]
        rl_stop_line = 90

        # Stop line waypoint (where speed = 0.0)
        rl_stop_line_wp = (self.red_light_wp - rl_stop_line + waypoints_num) % waypoints_num

        # Deceleration length in wp (before red light)
        # decel_len = 130


        uniform_speed = True
        if self.red_light_wp < 0:
            # There is no red light ahead, so just set up max speed
            uniform_speed = True
            target_speed = TARGET_SPEED
            self.slowing_down = False
        elif wps_to_light > LOOKAHEAD_WPS:
            # Red light is farther than number of points to return,
            # so again use max speed for all waypoints
            uniform_speed = True
            target_speed = TARGET_SPEED
            self.slowing_down = False
        elif wps_to_light < rl_stop_line_nearest and not self.slowing_down:
            # Red light is too close for us to stop, move forward
            uniform_speed = True
            target_speed = TARGET_SPEED
        elif wps_to_light < rl_stop_line:
            # Red light is already too close, stop our car quickly
            uniform_speed = True
            target_speed = 0.0
            self.slowing_down = True
        else:
            # Red light is ahead, need to change speed gradually
            uniform_speed = False

        if uniform_speed:
            wp_i = wp_next
            while wp_i != la_wp:
                waypoint = self.waypoints[wp_i]
                waypoint.twist.twist.linear.x = target_speed
                final_waypoints.append(waypoint)
                wp_i = (wp_i + 1) % waypoints_num
        else:
            # wp_next -- rl_stop_line_wp -- red_light_wp -- la_wp

            self.slowing_down = True

            # dist wp_next -- rl_stop_line_wp
            dist_rl_stop = helper.wp_distance(wp_next, rl_stop_line_wp, self.waypoints)
            wp_i = wp_next
            rl_stop_reached = False
            while wp_i != la_wp:
                waypoint = self.waypoints[wp_i]

                if wp_i == rl_stop_line_wp:
                  rl_stop_reached = True

                wp_i_next = (wp_i + 1) % waypoints_num

                if not rl_stop_reached:
                  # v = sqrt(2 * dist * max_decel)

                  #   speed = math.sqrt(2 * dist_rl_stop * MAX_DECEL)
                  speed = TARGET_SPEED
                  waypoint.twist.twist.linear.x = min(TARGET_SPEED, speed)

                  dist_rl_stop -= helper.wp_distance(wp_i, wp_i_next, self.waypoints)
                else:
                  waypoint.twist.twist.linear.x = 0.0

                final_waypoints.append(waypoint)
                wp_i = wp_i_next


        '''
        wp_i = wp_next
        for i in range(LOOKAHEAD_WPS):

            waypoint = self.waypoints[wp_i]

            waypoint.twist.twist.linear.x = TARGET_SPEED

            # Before red light set dec_len waypoints velocity to 0.0
            if self.red_light_wp > -1:
                if i > wps_to_light - decel_len:
                    waypoint.twist.twist.linear.x = 0.0

            final_waypoints.append(waypoint)
            wp_i = (wp_i + 1) % waypoints_num
        '''


        log_out = (self.cnt % 20 == 0)

        # rospy.loginfo("one point = {}".format(self.waypoints[0]))

        # orientation = self.waypoints[closest_waypoint].pose.pose.orientation

        if log_out:
            # rospy.loginfo('final_waypoints[0] = {}'.format(final_waypoints[0]))
            rospy.loginfo("pose x, y, yaw = {}, {}, {}".format(pose.pose.position.x,
                pose.pose.position.y, helper.yaw_from_orientation(pose.pose.orientation)))
            # rospy.loginfo("next wp x, y   = {}, {}".format(final_waypoints[0].pose.pose.position.x,
            #     final_waypoints[0].pose.pose.position.y))
            # rospy.loginfo("next wp linear.x   = {}".format(final_waypoints[0].twist.twist.linear.x))
            rospy.loginfo('wp_next = {}'.format(wp_next))
            rospy.loginfo('red_light_wp = {}'.format(self.red_light_wp))
            # rospy.loginfo('dist to zero = {}'.format(wps_to_light - decel_len))
            # rospy.loginfo('len wp = {}'.format(len(final_waypoints)))

            # rospy.loginfo("dist min = [{}] = {}".format(closest_waypoint, dists[closest_waypoint]))
            # rospy.loginfo("yaw = {}".format(helper.yaw_from_orientation(orientation)))

            # speed_list = [str(w.twist.twist.linear.x) for w in final_waypoints]
            # rospy.loginfo("final_waypoints[{}] = [{}]".format(len(final_waypoints), ",".join(speed_list)))

        self.cnt += 1

        self.publish(final_waypoints)

    def publish(self, waypoints):
      lane = Lane()
      lane.header.frame_id = '/world'
      lane.header.stamp = rospy.Time(0)
      lane.waypoints = waypoints
      self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        # rospy.loginfo('received waypoints len = {}'.format(len(waypoints.waypoints)))
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_wp = int(msg.data)
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
