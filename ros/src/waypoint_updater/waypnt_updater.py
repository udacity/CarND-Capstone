#!/usr/bin/env python

# File renamed to waypnt_updater.py because original name
# waypoint_updater.py which matches directory and package name
# prevented loading from waypoint_updater.cfg

import rospy
import math
import copy
import numpy as np
from scipy.spatial import KDTree
from jmt import JMT, JMTDetails, JMTD_waypoint

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Bool, String, Int32
from dynamic_reconfigure.server import Server
from waypoint_updater.cfg import DynReconfConfig

'''
This node will publish waypoints from the car's current position to some `x`
distance ahead.  As mentioned in the doc, you should ideally first implement
a version which does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of
traffic lights too.

Please note that our simulator also provides the exact location of traffic
lights and their current status in `/vehicle/traffic_lights` message. You
can use this message to build this node as well as to verify your TL
classifier.

'''


def get_accel_distance(Vi, Vf, A_avg, Ai=0.0):
    if math.fabs(A_avg) < 0.01:
        # avoid divide by 0 - should not happen
        rospy.logwarn("very low acceleration {} used in get_accel_distance"
                      .format(A_avg))
        return 50.0  # relatively long distance

    distance = (Vf**2 - Vi**2)/(2.0 * A_avg)
    dist_inc = 0.0
    # no easy way to calculate this - just add a little bias
    # if initial acceleration in opposite direction than needed
    # then add extra distance
    if Ai != 0.0:
        dist_inc = -math.fabs(A_avg)/A_avg * Ai

    total = math.fabs(distance + dist_inc)
    rospy.logdebug("get accel_distance Vi={:3.2f}m/s, Ai={:3.2f}m/s^2, Vf={:3.2f}m/s,"
                   " distance={:3.2f}m, added_inc={:3.2f}m, result={:3.2f}m"
                   .format(Vi, Ai, Vf, distance, dist_inc, total))
    return total


def get_accel_time(S, Vi, Vf):
    if (Vi + Vf) < 0.25:
        # avoid divide by 0
        return (0.0)
    return math.fabs((2.0 * S / (Vi + Vf)))


class WaypointUpdater(object):
    def __init__(self):
        self.testing = False
        # self.testing = True
        self.test_counter = 0
        self.dyn_vals_received = False
        self.waypoints = []
        self.pose = None
        self.velocity = None
        self.stop_calc_v = 0.0  # last measurement used to calc stopping distance
        self.stop_calc_a = 0.0  # last measurement used to calc stopping distance
        self.stop_target = None
        self.lights = None
        self.final_waypoints = []
        self.waypoints_2d = []  # store x,y pairs for use in KDTree
        self.waypoint_tree = None  # store KDTree in here
        self.final_waypoints_start_ptr = 0
        self.back_search = False
        self.last_search_distance = None
        self.last_search_time = None
        self.dyn_test_stoplight = False
        self.next_tl_wp = 0 #-1  # None
        self.next_tl_wp_tmp = 0 # use to prevent race conditions during loop cycle
        self.dyn_tl_buffer = 3.5  # tunable distance to stop before tl wp
        self.dyn_creep_zone = 7.5  # should only creep forward in this buffer
        self.dyn_buffer_offset = 2.0
        # self.dyn_jmt_time_factor = 1.0  # tunable factor to make nicer s curve
        self.update_rate = 10
        self.max_velocity = 0.0  # set based on max velocity in waypoints
        self.default_velocity = 0.0 #10.7
        self.lookahead_wps = self.lookahead_wps_reset = 20  # 200 is too many
        self.subs = {}
        self.pubs = {}
        self.dyn_reconf_srv = None
        self.max_s = 0.0  # length of track
        self.JMT_List = []
        self.default_accel = 0.8
        self.state = 'stopped'  # for now only use to see if stopped or moving
        self.is_decelerating = True
        self.min_stop_distance = 0.0
        self.stopping_distance = 0.0
        self.decel_at_min_moving_velocity = -0.1  # had biased to -0.5 but not needed
        self.got_to_end = False  # have we reached the end of the track?
        self.dbw_enabled = False

        rospy.init_node('waypoint_updater', log_level=rospy.INFO)
        #rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        self.initial_accel = rospy.get_param('~initial_accel')  # 0.3
        self.max_accel = rospy.get_param('~max_accel')  # 5.0
        self.max_jerk = rospy.get_param('~abs_max_jerk')  # 5.0
        self.max_desired_jerk = rospy.get_param('~max_desired_jerk')  # 1.0
        self.handoff_velocity = rospy.get_param('~handoff_velocity')  # 1.0
        self.min_moving_velocity = rospy.get_param('~min_moving_velocity')  #1.0

        self.subs['/base_waypoints'] = \
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.subs['/current_pose'] = \
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.subs['/current_velocity'] = \
            rospy.Subscriber('/current_velocity', TwistStamped,
                             self.velocity_cb)

        self.subs['/traffic_waypoint'] = \
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.subs['/vehicle/dbw_enabled'] = \
            rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # TODO: Add a subscriber for /obstacle_waypoint

        self.pubs['/final_waypoints'] = rospy.Publisher('/final_waypoints',
                                                        Lane, queue_size=1)

        # publish a boolean for acceleration vs deceleration intention
        self.pubs['/is_decelerating'] = rospy.Publisher('/is_decelerating',
                                                        Bool, queue_size=1)

        # self.dyn_reconf_srv = Server(DynReconfConfig, self.dyn_vars_cb)
        # is called from waypoints_cb so no collision between them

        self.loop()

    def loop(self):
        self.rate = rospy.Rate(self.update_rate)
        # wait for waypoints and pose to be loaded before trying to update
        # waypoints
        while not self.waypoints:
            self.rate.sleep()
        while not self.pose:
            self.rate.sleep()
        while not rospy.is_shutdown():

            if self.waypoints and self.waypoints_2d and self.waypoint_tree:
                self.send_waypoints()
            self.rate.sleep()

    # adjust dynamic variables
    def dyn_vars_cb(self, config, level):
        self.dyn_vals_received = True
        if self.update_rate:
            old_update_rate = self.update_rate
            old_default_velocity = self.default_velocity
            old_default_accel = self.default_accel
            old_lookahead_wps = self.lookahead_wps
            old_test_stoplight_wp = self.next_tl_wp_tmp
            old_tl_buffer = self.dyn_tl_buffer
            old_creep_zone = self.dyn_creep_zone
            old_buffer_offset = self.dyn_buffer_offset
            # old_jmt_time_factor = self.dyn_jmt_time_factor

        rospy.loginfo("Received dynamic parameters {} with level: {}"
                      .format(config, level))

        if old_update_rate != config['dyn_update_rate']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting update Rate "
                          "from {} to {}".format(old_update_rate,
                                                 config['dyn_update_rate']))
            self.update_rate = config['dyn_update_rate']
            self.rate = rospy.Rate(self.update_rate)

        if old_default_velocity != config['dyn_default_velocity']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting default_"
                          "velocity from {:3.2f}m/s to {:3.2f}m/s"
                          .format(old_default_velocity,
                                  config['dyn_default_velocity']))

            if config['dyn_default_velocity'] > self.max_velocity:
                rospy.logwarn("waypoint_updater:dyn_vars_cb default_velocity "
                              "limited to max_velocity {:3.2f}m/s"
                              .format(self.max_velocity))
                self.default_velocity = self.max_velocity * 0.975
            else:
                self.default_velocity = config['dyn_default_velocity']

        if old_default_accel != config['dyn_default_accel']:
            rospy.logwarn("waypoint_updater:dyn_vars_cb Adjusting default_"
                          "accel from {:3.2f}m/s^2 to {:3.2f}m/s^2"
                          .format(old_default_accel,
                                  config['dyn_default_accel']))
            self.default_accel = config['dyn_default_accel']

        if old_lookahead_wps != config['dyn_lookahead_wps']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting lookahead_"
                          "wps from {} to {}"
                          .format(old_lookahead_wps,
                                  config['dyn_lookahead_wps']))
            self.lookahead_wps = config['dyn_lookahead_wps']
            self.lookahead_wps_reset = self.lookahead_wps

        if old_test_stoplight_wp != config['dyn_test_stoplight_wp']\
                and\
                config['dyn_test_stoplight_wp'] != -1:
            self.dyn_test_stoplight = True
            # only update if dyn_test_stoplight_wp not set to -1
            rospy.logwarn("waypoint_updater:dyn_vars_cb Adjusting next "
                          "stoplight from {} to {}"
                          .format(old_test_stoplight_wp,
                                  config['dyn_test_stoplight_wp']))
            rospy.logwarn("Ignore /traffic_waypoint message while testing.")
            self.next_tl_wp_tmp = min(config['dyn_test_stoplight_wp'],
                                  len(self.waypoints)-1)
        else:
            self.dyn_test_stoplight = False

        if old_tl_buffer != config['dyn_tl_buffer']:
            rospy.loginfo("dyn_vars_cb Adjusting tl_buffer"
                          "from {:3.2f}m to {:3.2f}m"
                          .format(old_tl_buffer,
                                  config['dyn_tl_buffer']))
            self.dyn_tl_buffer = config['dyn_tl_buffer']

        if old_buffer_offset != config['dyn_buffer_offset']:
            rospy.loginfo("dyn_vars_cb Adjusting buffer_offset"
                          "from {:3.2f}m to {:3.2f}m"
                          .format(old_buffer_offset,
                                  config['dyn_buffer_offset']))
            self.dyn_buffer_offset = config['dyn_buffer_offset']
            if self.dyn_buffer_offset > self.dyn_tl_buffer:
                rospy.logwarn("dyn_buffer_offset={:3.2f}m set larger"
                              "than dyn_tl_buffer={:3.2}m".format(
                                  self.dyn_buffer_offset,
                                  self.dyn_tl_buffer
                              ))


        if old_creep_zone != config['dyn_creep_zone']:
            rospy.loginfo("dyn_vars_cb Adjusting creep_zone "
                          "from {:3.2f}m to {:3.2f}m"
                          .format(old_creep_zone,
                                  config['dyn_creep_zone']))
            self.dyn_creep_zone = config['dyn_creep_zone']

        # if old_jmt_time_factor != config['dyn_jmt_time_factor']:
        #     rospy.loginfo("dyn_vars_cb Adjusting jmt_time_factor"
        #                   "from {:3.2f} to {:3.2f}"
        #                   .format(old_tl_buffer,
        #                           config['dyn_jmt_time_factor']))

        #     self.dyn_jmt_time_factor = config['dyn_jmt_time_factor']

        return config

    def velocity_cb(self, twist_msg):
        # Check this is right
        self.velocity = twist_msg.twist.linear.x
        rospy.logdebug("Velocity reported as {:2.3f} mps".format(self.velocity))

    def pose_cb(self, pose_msg):
        self.pose = pose_msg.pose
        rospy.logdebug("waypoint_updater:pose_cb pose set to  %s", self.pose)

    # Load set of waypoints from /basewaypoints into self.waypoints
    # this should only happen once, so we unsubscribe at end
    def waypoints_cb(self, lane_msg):

        rospy.loginfo("waypoint_updater:waypoints_cb loading waypoints")
        if not self.waypoints:
            cntr = 0
            s = 0.0
            max_velocity = 0.0
            wpt = None  # prevents linter from complaining
            t_waypoints = []
            t_waypoints_2d = []

            for lanemsg_wpt in lane_msg.waypoints:

                if cntr > 0 and wpt:
                    # won't come into here until after wpt loaded
                    # in previous loop
                    s += math.sqrt((wpt.get_x() - lanemsg_wpt.pose.pose.
                                    position.x)**2 +
                                   (wpt.get_y() - lanemsg_wpt.pose.pose.
                                    position.y)**2)

                wpt = JMTD_waypoint(lanemsg_wpt, cntr, s)
                t_waypoints.append(wpt)
                t_waypoints_2d.append([wpt.get_x(), wpt.get_y()])

                if max_velocity < wpt.get_maxV():
                    max_velocity = wpt.get_maxV()
                cntr += 1

            self.waypoints = t_waypoints
            self.waypoints_2d = t_waypoints_2d
            self.waypoint_tree = KDTree(self.waypoints_2d)

            rospy.loginfo("waypoints_cb {} waypoints loaded, last waypoint "
                          "ptr_id = {} at s= {:3.3f}".
                          format(len(self.waypoints), self.waypoints[cntr-1].
                                 ptr_id, self.waypoints[cntr-1].get_s()))
            self.max_s = self.waypoints[cntr-1].get_s()
            # setting max velocity based on project requirements in
            # Waypoint Updater Node Revisited
            self.max_velocity = max_velocity
            rospy.loginfo("waypoint_updater:waypoints_cb max_velocity set to "
                          " {:3.3f} based on max value in waypoints."
                          .format(self.max_velocity))

            # now max_velocity is known, set up dynamic reconfig
            if not self.dyn_reconf_srv:
                self.dyn_reconf_srv = Server(DynReconfConfig, self.dyn_vars_cb)
                rospy.loginfo("dynamic_parm server started")

        else:
            rospy.logerr("waypoint_updater:waypoints_cb attempt to load "
                         "waypoints when we have already loaded {} waypoints"
                         .format(len(self.waypoints)))

        self.subs['/base_waypoints'].unregister()
        rospy.loginfo("Unregistered from /base_waypoints topic")

    # Receive a msg from /traffic_waypoint about the next stop line
    def traffic_cb(self, traffic_msg):
        if traffic_msg.data != self.next_tl_wp_tmp:
            if self.dyn_test_stoplight is False:
                self.next_tl_wp_tmp = traffic_msg.data
                rospy.logwarn("new /traffic_waypoint message received is wp: {}, "
                          "while car is at wp {}".format(self.next_tl_wp_tmp,
                          self.final_waypoints_start_ptr))
            else:
                rospy.logdebug("Ignoring /traffic_waypoint message while testing.")
        # else:
            # just for debug to see what we're getting
            rospy.logdebug("same /traffic_waypoint message tl_wp = {} received."
                           .format(traffic_msg.data))

    def dbw_enabled_cb(self, dbw_enabled_message):
        if self.dbw_enabled != dbw_enabled_message.data:
            rospy.logwarn("Setting dbw to {} in waypoint_updater"
                          .format(dbw_enabled_message.data))
            self.dbw_enabled = dbw_enabled_message.data

    def get_dist_to_tl(self):
        # this can happen before we get a traffic_wp msg
        if not self.next_tl_wp:
            self.next_tl_wp = -1
        if self.next_tl_wp in range(1, len(self.waypoints)):
            dist_to_tl = self.waypoints[self.next_tl_wp - 1].get_s() -\
                        self.waypoints[self.final_waypoints_start_ptr].get_s()
        else:
            dist_to_tl = 5000  # big number
        return dist_to_tl

    def get_min_stopping_distance(self, ptr_id, max_neg_jerk):
        # Use JMT to figure out shortest stopping distance with
        # jerk at both ends of slowdown compared to jerk limit

        curpt = self.waypoints[ptr_id]
        if curpt.get_v() <= self.min_moving_velocity:
            return 0.5

        timer_start = rospy.get_time()

        a = curpt.get_a()

        if a > 0.0:
            # currently speeding up
            time_factor = 0.8
        else:
            time_factor = 1.0 # self.dyn_jmt_time_factor

        if math.fabs(-self.max_desired_jerk - max_neg_jerk) < 0.1:
            # if we are using_max_desired_jerk then start here
            decel_rate = 0.6
        else:
            # empirical settings to start closer to goal
            if a < 1.0:
                decel_rate = 2.0
            elif a < 2.0:
                decel_rate = 1.5
            elif a < 3.0:
                decel_rate = 1.35
            elif a < 3.4:
                decel_rate = 1.2
            else:
                decel_rate = 1.0

        a_dist = get_accel_distance(curpt.JMTD.V, self.min_moving_velocity*1.25, decel_rate,
                                    curpt.JMTD.A)

        too_short = False

        T = get_accel_time(a_dist, curpt.JMTD.V, 0.0) * time_factor
        start = [curpt.JMTD.S, curpt.JMTD.V, curpt.JMTD.A]
        end = [curpt.JMTD.S + a_dist, self.min_moving_velocity*1.25, self.decel_at_min_moving_velocity]
        rospy.logdebug("Test decel_rate {:3.3f} from v={:3.3f}, a={:3.3f} to v={:3.3f}, a={:3.3f} "
                       " in dist {:3.3f} m in time {:3.3f} s"
                       .format(decel_rate, start[1], start[2], end[1], end[2], a_dist, T))

        jmt = JMT(start, end, T)
        start_jerk = jmt.get_j_at(0.1)
        end_jerk = jmt.get_j_at(T - 0.1)
        acc = jmt.get_a_at(T - 0.5)

        rospy.logdebug("found initial jerk={:3.2f}m/s^3, final jerk={:3.2f}m/s^3 "
                       "final acc of {:3.2f}m/s^2 "
                       "when using decel_rate={:3.2f}m/s a_dist ={:3.2f}m and T={:3.2f}s"
                       .format(start_jerk, end_jerk, acc, decel_rate, a_dist, T))

        if start_jerk < max_neg_jerk or end_jerk > -max_neg_jerk:
            too_short = True
            dist_diff = 1.0
        else:
            dist_diff = -1.0
        if math.fabs(max_neg_jerk + self.max_desired_jerk) < 0.1:
            dist_diff = 2 * dist_diff

        optimized = False
        counter = 0
        while optimized is False:
            counter += 1
            old_sjerk = start_jerk
            old_ejerk = end_jerk
            a_dist = a_dist + dist_diff
            if a_dist < 0.0:
                final_dist = a_dist - dist_diff
                duration = rospy.get_time() - timer_start
                rospy.logdebug("Shortest distance to decelerate with max_neg_jerk"
                        "={:3.3f} from v={:3.3f}, a={:3.3f} to v={:3.3f} in "
                        "dist {:3.3f}m in time {:3.3f}s, took {:3.4f}s to calc"
                        .format(start_jerk, start[1], start[2], end[1],
                                final_dist, T, duration))
                return(final_dist)

            end[0] = curpt.get_s() + a_dist
            T = get_accel_time(a_dist, curpt.JMTD.V, 0.0) * time_factor

            jmt = JMT(start, end, T)
            start_jerk = jmt.get_j_at(0.1)
            end_jerk = jmt.get_j_at(T - 0.1)
            acc = jmt.get_a_at(T - 0.5)

            rospy.logdebug("found initial jerk={:3.2f}m/s^3, final jerk={:3.2f}m/s^3 "
                       "final acc of {:3.2f}m/s^2 "
                       "when using decel_rate={:3.2f}m/s a_dist ={:3.2f}m and T={:3.2f}s"
                       .format(start_jerk, end_jerk, acc, decel_rate, a_dist, T))

            if too_short is True:
                # looking for first instance that matches
                if start_jerk > max_neg_jerk and end_jerk < -max_neg_jerk:
                    final_dist = a_dist
                    optimized = True
            else:
                # looking for first instance that fails
                # this is wrong, but fixing it breaks everything
                #if start_jerk < 0.0 - max_neg_jerk:
                if start_jerk < max_neg_jerk or end_jerk > -max_neg_jerk:
                    final_dist = a_dist - dist_diff
                    start_jerk = old_sjerk
                    end_jerk = old_ejerk
                    optimized = True

            if counter > 50 and optimized is False:
                final_dist = a_dist
                rospy.logwarn("counter is {} in get_min_stopping_distance initial jerk={:3.2f}, final_jerk={:3.2f} dist={:3.2f}m- bail!"
                              .format(counter, start_jerk, end_jerk, final_dist))
                optimized = True

        duration = rospy.get_time() - timer_start
        rospy.loginfo("get_min_stopping_distance found shortest Distance of {:3.3f}m to decelerate with start_jerk={:3.3f}, end_jerk={:3.3f} ,"
                      "from v={:3.3f}, a={:3.3f} to v={:3.3f}, a={:3.3f}"
                      " in {:3.3f}s - Took {:3.4f}s to calc"
                      .format(final_dist, start_jerk, end_jerk, start[1], start[2], end[1], end[2],
                              T, duration))
        return final_dist

    def get_stopping_time(self, start, end):
        # Use JMT to figure out proper time for deceleration where
        # curve does not wobbble below min_moving _velocity
        # start[s, velocity, acc]
        # end[s, velocity, acc]

        save_start = copy.deepcopy(start)
        save_end = copy.deepcopy(end)

        if start[1] <= self.min_moving_velocity:
            return 0.5

        timer_start = rospy.get_time()

        if start[2] > 0.0:
            # currently speeding up
            time_factor = 0.8
        else:
            time_factor = 1.0 # self.dyn_jmt_time_factor

        a_dist = end[0] - start[0]

        T = get_accel_time(a_dist, start[1], end[1]) * time_factor

        rospy.logdebug("Test decel from v={:3.3f}m/s, a={:3.3f}m/s^2 to v={:3.3f}m/s, a={:3.3f}m/s^2"
                       " in {:3.3f}m and {:3.3f}s."
                       .format(start[1], start[2], end[1], end[2], a_dist, T))

        Tmin = (end[0] - start[0]) / self.default_velocity

        save_T = T
        try_reverse = False
        jmt = JMT(start, end, T)
        jerk = jmt.get_j_at(0.1)
        e_acc = jmt.get_a_at(T - 0.5)
        s_acc = jmt.get_a_at(0.5)

        rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and time of {:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        optimized = False
        time_diff = T * 0.01
        counter = 0
        while optimized is False:
            old_e_acc = e_acc
            old_s_acc = s_acc
            if e_acc < end[2] and s_acc < start[2]:
                optimized = True
            else:
                counter = counter + 1
                T = T + time_diff
                jmt = JMT(start, end, T)
                jerk = jmt.get_j_at(0.1)
                e_acc = jmt.get_a_at(T - 0.5)
                s_acc = jmt.get_a_at(0.5)

                rospy.logdebug("e_acc={:3.2f}, e_acc-end2={:3.2f}, old_eacc-end2={:3.2f} ,s_acc={:3.2f}, s_acc-start2={:3.2f}, old_sacc-start2={:3.2f}"
                        .format(e_acc, e_acc-end[2], old_e_acc-end[2], s_acc, s_acc-start[2],old_s_acc-start[2]))
                if counter == 1 and\
                        ((e_acc > end[2] and math.fabs(e_acc - end[2]) > math.fabs(old_e_acc - end[2])) or
                        (s_acc > start[2] and math.fabs(start[2] - s_acc) > math.fabs(start[2] - old_e_acc))):
                    rospy.loginfo("searching wrong direction - go the other way")
                    time_diff = 0.0 - time_diff

                if counter > 50 or T < Tmin:
                    rospy.logwarn("counter is {} in get_stopping_time - bail on searching in this direction!"
                                .format(counter))
                    optimized = True
                    try_reverse = True
                rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and T={:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        if try_reverse == False:
            duration = rospy.get_time() - timer_start
            rospy.loginfo("get_stopping_time found shortest distance to decelerate with max_neg_jerk={:3.3f}m/s^3 "
                            "from v={:3.3f}m/s, a={:3.3f}m/s^2 to v={:3.3f}m/s a={:3.3f}m/s^2 in dist {:3.3f}m"
                            " in time {:3.3f}s - Took {:3.4f}s to calc."
                            .format(jerk, start[1], start[2], end[1], end[2],
                                    a_dist, T, duration))
            return T

        #########
        T = save_T
        start = save_start
        end = save_end
        jmt = JMT(start, end, T)
        jerk = jmt.get_j_at(0.1)
        e_acc = jmt.get_a_at(T - 0.5)
        s_acc = jmt.get_a_at(0.5)

        rospy.logdebug("Reverse found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and time of {:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        optimized = False
        time_diff = 0.0 - time_diff

        counter = 0
        while optimized is False:
            old_e_acc = e_acc
            old_s_acc = s_acc
            if e_acc < end[2] and s_acc < start[2]:
                optimized = True
            else:
                counter = counter + 1
                T = T + time_diff
                jmt = JMT(start, end, T)
                jerk = jmt.get_j_at(0.1)
                e_acc = jmt.get_a_at(T - 0.5)
                s_acc = jmt.get_a_at(0.5)
                rospy.logdebug("e_acc={:3.2f}, e_acc-end2={:3.2f}, old_eacc-end2={:3.2f} ,s_acc={:3.2f}, s_acc-start2={:3.2f}, old_sacc-start2={:3.2f}"
                        .format(e_acc, e_acc-end[2], old_e_acc-end[2], s_acc, s_acc-start[2],old_s_acc-start[2]))

                if counter > 50 or T < Tmin:
                    rospy.logwarn("counter is {} in get_stopping_time - bail!"
                                .format(counter))
                    optimized = True
                rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and T={:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        duration = rospy.get_time() - timer_start
        rospy.loginfo("get_stopping_time found shortest distance to decelerate with max_neg_jerk={:3.3f}m/s^3 "
                        "from v={:3.3f}m/s, a={:3.3f}m/s^2 to v={:3.3f}m/s a={:3.3f}m/s^2 in dist {:3.3f}m"
                        " in time {:3.3f}s - Took {:3.4f}s to calc."
                        .format(jerk, start[1], start[2], end[1], end[2],
                                a_dist, T, duration))
        return T

    def setup_stop_jmt(self, ptr_id, a_dist):
        # this is set up to stop the car in a desired distance

        curpt = self.waypoints[ptr_id]
        target_velocity = 1.25 * self.min_moving_velocity # was 1.1, move to just above creep speed
        if curpt.get_a() > 0.0:
            time_adjustment = 0.8
        else:
            time_adjustment = 1.0  # self.dyn_jmt_time_factor

        T = get_accel_time(a_dist, curpt.JMTD.V, target_velocity) *\
            time_adjustment

        if a_dist < 0.1 or T < 0.1:
            # dummy values to prevent matrix singularity
            # if no velocity change required
            rospy.loginfo("No change in velocity found in setup_stop_jmt.")
            a_dist = 0.1
            T = 0.1

        start = [curpt.JMTD.S, curpt.JMTD.V, curpt.JMTD.A]
        end = [curpt.JMTD.S + a_dist, target_velocity, self.decel_at_min_moving_velocity]
        T = self.get_stopping_time(start, end)

        rospy.loginfo("Car set to decel from v={:3.3f}, a={:3.3f} to v={:3.3f}, a={:3.3f}"
            " in dist {:3.3f}m in time {:3.3f}s"
            .format(start[1], start[2], end[1], end[2], a_dist, T))

        jmt = JMT(start, end, T)
        self.JMT_List.append(jmt)
        jmt_ptr = len(self.JMT_List)
        return jmt_ptr-1

    def set_stopped(self, start_ptr, num_wps):
        # set V = 0 for waypoints in range
        if self.state != 'stopped':
            rospy.logwarn("Set car state to stopped at ptr = {}".format(start_ptr))
            self.state = 'stopped'
        for ptr in range(start_ptr, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            self.waypoints[mod_ptr].JMTD.set_VAJt(0.0, 0.0, 0.0, 0.0)
            self.waypoints[mod_ptr].set_v(0.0)
            self.waypoints[mod_ptr].JMT_ptr = -1
        # cleanup jmt stack
        self.JMT_List[:] = []
        # won't match self.next_tl_wp when reset
        self.stop_target = 0

    def set_transition_to_stop(self, mod_ptr):
        # gracefully slow down to stopped at end of jmt decel curve
        # due to lack of control at very low speeds gracefulness was removed
        disp = self.waypoints[mod_ptr].get_s() -\
            self.waypoints[mod_ptr-1].get_s()

        decel = min(self.waypoints[mod_ptr-1].JMTD.A * 0.5, -0.25)

        #velocity = min(max(0.0, self.waypoints[mod_ptr-1].get_v() -
        #                   math.sqrt(math.fabs(decel) * disp * 2.0)),
        #               self.waypoints[mod_ptr].get_maxV())
        #if velocity <= self.min_moving_velocity:
        if self.velocity <= self.min_moving_velocity:
            if self.get_dist_to_tl() < self.dyn_tl_buffer:
                velocity = 0.0
                decel = 0.0
            else:
                velocity = self.min_moving_velocity
                decel = 0.0
        else:
            velocity = self.min_moving_velocity

        self.waypoints[mod_ptr].set_v(velocity)
        self.waypoints[mod_ptr].JMT_ptr = -1
        self.waypoints[mod_ptr].JMTD.set_VAJt(
            velocity, decel, 0.0, 0.0)

    def gen_point_in_jmt_curve(self, pt, jmt_ptr, t):
        # wrapper function to get the JMT details at the displacement
        # of the waypoint and adjust the waypoint values accordingly
        pt.JMT_ptr = jmt_ptr
        JMT_instance = self.JMT_List[jmt_ptr]
        jmt_pnt = JMT_instance.JMTD_at(pt.get_s(), t, JMT_instance.T * 1.5)
        if jmt_pnt is None:
            rospy.logwarn("JMT_at returned None at ptr_id = {}"
                          .format(pt.ptr_id))
            return -1

        pt.JMTD.set_VAJt(jmt_pnt.V, jmt_pnt.A, jmt_pnt.J, jmt_pnt.time)
        pt.set_v(jmt_pnt.V)
        return jmt_pnt.time

    def produce_slowdown(self, start_ptr, num_wps, distance):
        # this generates the deceleration curve based on JMT
        # for the range of waypoints, inserting V = 0.0
        # beyond the stop point
        curpt = self.waypoints[start_ptr]
        recalc = False

        if curpt.JMT_ptr == -1 or self.state != 'slowdown':
            jmt_ptr = self.setup_stop_jmt(start_ptr, distance)
            curpt.JMT_ptr = jmt_ptr
            if self.state != 'slowdown':
                rospy.logwarn("Set car state to slowdown at ptr = {}".format(curpt.ptr_id))
                self.state = 'slowdown'
        else:
            jmt_ptr = curpt.JMT_ptr
            rospy.logdebug("using old jmt_ptr = {}".format(jmt_ptr))
        JMT_instance = self.JMT_List[jmt_ptr]

        t = 0.0
        for ptr in range(start_ptr + 1, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            if curpt.get_s() <= JMT_instance.final_displacement:
                # create the main part of the jmt curve
                if curpt.JMT_ptr != jmt_ptr:
                    t = self.gen_point_in_jmt_curve(curpt, jmt_ptr, t)
                    if self.check_point(curpt) is True:
                        recalc = True
            else:
                # the car has reached the point where the JMT
                # curve should reach target velocity of 0.0
                # we smooth the transition
                rospy.logdebug("{:4.2f} beyond S = {:4.2f} at ptr_id = {}"
                               .format(curpt.get_s(),
                                       JMT_instance.final_displacement,
                                       mod_ptr))
                self.set_transition_to_stop(mod_ptr)
        return recalc

    def maintain_speed(self, start_ptr, num_wps):
        # sets the speed of the car to default velocity for the
        # range of waypoints
        for ptr in range(start_ptr, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            rospy.logdebug("setting to default velocity {:4.3f} at ptr = {}"
                           .format(self.default_velocity, mod_ptr))

            velocity = min(curpt.get_maxV(), self.default_velocity)
            curpt.set_v(velocity)
            if velocity > 0.0:
                try:
                    del_t += (curpt.get_s() - self.waypoints[(ptr - 1) %
                                len(self.waypoints)].get_s()) / velocity
                except NameError:
                    del_t = 0.0
            else:
                del_t = 0
            curpt.JMTD.set_VAJt(velocity, 0.0, 0.0, del_t)
            curpt.JMT_ptr = -1

    def init_acceleration(self, start_ptr, num_wps):
        # starts the car moving 
        # sim doesn't handle speeds below 1.0 well 
        # - just set to 1.0 to start off.
        # no need to increment offset in this case
        offset = 0

        velocity = max(self.min_moving_velocity,
                       self.waypoints[start_ptr].get_v())
        self.waypoints[start_ptr].set_v(velocity)
        self.waypoints[start_ptr].JMTD.set_VAJt(
                    velocity, self.initial_accel, 0.0, 0.0)
        return offset

    def generate_speedup(self, start_ptr, end_ptr):
        # this is triggered after initiate_aceleration()
        # gets the car moving - that way we can adjust them
        # independently
        recalc = False
        curpt = self.waypoints[start_ptr]

        if curpt.JMT_ptr == -1 or self.state != 'speedup':
            rospy.logwarn("Set car state to speedup at ptr={}".format(start_ptr))
            self.state = 'speedup'
            accel_rate, a_dist, T = self.get_max_accel(curpt.ptr_id)
            curpt.JMT_ptr = self.setup_speedup_jmt(curpt, a_dist, self.default_velocity, T)
            jmt_ptr = curpt.JMT_ptr
        else:
            jmt_ptr = curpt.JMT_ptr
            rospy.logdebug("using old jmt_ptr = {}".format(curpt.JMT_ptr))
        JMT_instance = self.JMT_List[jmt_ptr]

        t = 0.0
        for ptr in range(start_ptr+1, end_ptr):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            if curpt.get_s() <= JMT_instance.final_displacement:
                # create the main part of the jmt curve
                if curpt.JMT_ptr != jmt_ptr:
                    t = self.gen_point_in_jmt_curve(curpt, jmt_ptr, t)
                    if self.check_point(curpt) is True:
                        recalc = True
            else:
                # The car has reached the point where it is near to target
                # velocity
                # rospy.logdebug("{} beyond S = {} at ptr_id = {}".format
                #               (curpt.get_s(), JMT_instance.final_displacement,
                #                mod_ptr))
                self.maintain_speed(mod_ptr, 1)
        return recalc

    def find_stopping_distances(self, ptr):
        # this function generates stopping_distance and min_stopping_distance
        # where stopping_distance is calculated with a comfortable
        # deceleration and min_stop_distance is at limits of acceptable jerk

        # this check is to see if stopping distances calculated in previous
        # won't still be valid
        if (math.fabs(self.stop_calc_v - self.waypoints[ptr].get_v()) > 0.1
              or math.fabs(self.stop_calc_a - self.waypoints[ptr].get_a()) > 0.1):
            # set the values used in the calc
            self.stop_calc_a = self.waypoints[ptr].get_a()
            self.stop_calc_v = self.waypoints[ptr].get_v()
            self.stopping_distance = self.get_min_stopping_distance(
                    self.final_waypoints_start_ptr, -self.max_desired_jerk)
            self.min_stop_distance = self.get_min_stopping_distance(
                    self.final_waypoints_start_ptr, -self.max_jerk)

    def set_creep(self, start_ptr, num_wps):
        # set V = 1.0 for waypoints in range
        if self.state != 'creeping':
            rospy.logwarn("Set car state to creeping at ptr = {}".format(start_ptr))
            self.state = 'creeping'
        for ptr in range(start_ptr, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            self.waypoints[mod_ptr].JMTD.set_VAJt(self.min_moving_velocity,
                                                  0.0, 0.0, 0.0)
            self.waypoints[mod_ptr].set_v(self.min_moving_velocity)
            self.waypoints[mod_ptr].JMT_ptr = -1

    def set_waypoints_velocity(self):
        offset = 0           # offset in front of car 
                             # - could use to account for latency
        recalc = False       # indication that JMT calcs exceed bounds

        if self.final_waypoints_start_ptr == len(self.waypoints) - 1:
            if self.got_to_end is False and self.dbw_enabled:
                rospy.logwarn("reached end of track at ptr = {}".format(self.final_waypoints_start_ptr))
                self.got_to_end = True
            dist_to_tl = 0.0
        else:
            dist_to_tl = self.get_dist_to_tl()

        # don't go beyond end of track
        if self.final_waypoints_start_ptr + self.lookahead_wps > len(self.waypoints)-1:
            self.lookahead_wps = (len(self.waypoints)) - self.final_waypoints_start_ptr

        if self.waypoints[self.final_waypoints_start_ptr].get_v() == 0.0:
            # we are stopped
            offset = 0
            if self.state != 'stopped':
                rospy.logwarn("Set car state to stopped at ptr = {}".format(self.final_waypoints_start_ptr))
                self.state = 'stopped'
                self.stopping_distance = 0.0
                self.min_stop_distance = 0.0

        # dont check distances when we are already trying to stop at the target
        if (self.state == 'slowdown' and self.stop_target == self.next_tl_wp) is False:
            self.find_stopping_distances(self.final_waypoints_start_ptr)
            rospy.loginfo("state = {}, dist_to_tl at tl_wp={} is {:4.3f}m, stopping_dist = {:4.3f}m, "
                      "min_stopping_distance = {:4.2f}m"
                      .format(self.state, self.next_tl_wp, dist_to_tl, self.stopping_distance,
                              self.min_stop_distance))
        else:
            rospy.loginfo("state = {}, dist_to_tl at tl_wp={} is {:4.3f}m"
                      .format(self.state, self.next_tl_wp, dist_to_tl))

        # handle case where car is stopped at lights and light is red
        # or keep in stopped state when dbw_enabled is OFF and car is stopped
        if ((self.state == 'stopped' and dist_to_tl < self.dyn_tl_buffer)
                or (self.dbw_enabled is False and self.velocity < self.min_moving_velocity)):
            self.set_stopped(self.final_waypoints_start_ptr,
                             self.lookahead_wps)

        # just creep up to red lights if stopped a short distance from them
        elif (self.state == 'stopped' or self.state == 'creeping' or\
             (self.state == 'speedup' and self.velocity < self.min_moving_velocity)) and\
               dist_to_tl < self.dyn_creep_zone + self.dyn_tl_buffer:

            if dist_to_tl > self.dyn_tl_buffer:
                self.set_creep(self.final_waypoints_start_ptr, self.lookahead_wps)
            else:
                self.set_stopped(self.final_waypoints_start_ptr, self.lookahead_wps)

        # stay in previously calculated slowdown
        elif self.state == 'slowdown' and self.stop_target == self.next_tl_wp:
            if dist_to_tl < self.dyn_tl_buffer and\
                    self.velocity < self.min_moving_velocity:
                self.set_stopped(self.final_waypoints_start_ptr, self.
                                 lookahead_wps)
            else:
                recalc = self.produce_slowdown(self.final_waypoints_start_ptr,
                                           self.lookahead_wps,
                                           dist_to_tl - (self.dyn_tl_buffer - self.dyn_buffer_offset))

        # switch to slowdown if possible!
        elif (self.state == 'speedup' or self.state == 'maintainspeed' or self.state == 'slowdown') and\
               dist_to_tl - self.dyn_tl_buffer < self.stopping_distance :
            if dist_to_tl - self.dyn_tl_buffer > self.min_stop_distance:
                if self.state != 'slowdown':
                    rospy.logwarn("Within stopping distance of TL - Start to slowdown!")
                self.stop_target = self.next_tl_wp
                recalc = self.produce_slowdown(self.final_waypoints_start_ptr,
                            self.lookahead_wps,
                            dist_to_tl - (self.dyn_tl_buffer - self.dyn_buffer_offset))
            else:
                rospy.logwarn("Distance to Red light {:3.2f}m shorter than ability {:3.2f}m to slow down in time at ptr = {}"
                                  .format(dist_to_tl, self.min_stop_distance, self.final_waypoints_start_ptr))
                if self.state == 'maintainspeed':
                    self.maintain_speed(self.final_waypoints_start_ptr, self.
                                        lookahead_wps)
                else:
                    recalc = self.generate_speedup(self.final_waypoints_start_ptr,
                        self.final_waypoints_start_ptr +
                        self.lookahead_wps)

        elif dist_to_tl > self.stopping_distance:
            if self.waypoints[self.final_waypoints_start_ptr].get_v() >= \
                min(self.default_velocity, self.waypoints[self.final_waypoints_start_ptr].get_maxV()):
                # handle case where car is at target speed and no
                # traffic lights within stopping distance
                if self.state != 'maintainspeed':
                    rospy.logwarn("Set car state to maintainspeed at ptr = {}".format(self.final_waypoints_start_ptr))
                    self.state = 'maintainspeed'
                self.maintain_speed(self.final_waypoints_start_ptr, self.
                                lookahead_wps)
            else:
                if self.waypoints[self.final_waypoints_start_ptr].get_v() <\
                        min(self.handoff_velocity, self.waypoints[self.final_waypoints_start_ptr].get_maxV()):
                    # Get the car going before handing off to JMT
                    if self.state != 'speedup':
                        rospy.loginfo("Get Car moving from very slow speed at ptr={}.".format(self.final_waypoints_start_ptr))
                    offset =\
                        self.init_acceleration(self.final_waypoints_start_ptr,
                                               self.lookahead_wps)
                if self.waypoints[self.final_waypoints_start_ptr].get_v() <\
                        self.waypoints[self.final_waypoints_start_ptr].get_maxV():
                    recalc = self.generate_speedup(self.final_waypoints_start_ptr +
                                               offset,
                                               self.final_waypoints_start_ptr +
                                               self.lookahead_wps)
        else:
            rospy.logerr("Fell out of state engine - in state {} with dist_to_tl={:3.2f}m,"
                         " min_stopping_distance={:3.2f}m, and min_stopping_distance={:3.2f}m"
                         .format(self.state, dist_to_tl, self.stopping_distance, self.min_stop_distance))

        if recalc is True:
            rospy.logwarn("recalc is set to {} we should recalculate"
                          .format(recalc))

        # Log JMT profile values - Vcalc is calculated value, wheras Veffective is value
        # in twist which might have been limited by local wpt.vMax or global self.max_velocity
        jmt_log = "\nptr_id, JMT_ptr, time, S, Vcalc, A, J, Veffective\n"
        for wpt in self.waypoints[self.final_waypoints_start_ptr:
                                  self.final_waypoints_start_ptr +
                                  self.lookahead_wps]:
            jmt_log += "{}, {}, {}, {:3.4f}\n".format(wpt.ptr_id, wpt.JMT_ptr, wpt.JMTD, wpt.get_v())
        rospy.loginfo(jmt_log)

    def setup_speedup_jmt(self, curpt, a_dist, target_velocity, acceltime=0):
        # this is set up to speedup the car to a desired velocity

        if curpt.get_a() < 0.0:
            time_adjustment = 1.1  # 1.5
        else:
            time_adjustment = 1.0  # self.dyn_jmt_time_factor

        if acceltime > 0.0:
            T = acceltime
        else:
            T = get_accel_time(a_dist, curpt.get_v(), target_velocity) *\
                time_adjustment

        if a_dist < 0.1 or T < 0.1:
            # dummy values to prevent matrix singularity
            # if no velocity change required
            rospy.logwarn("No change in velocity needed.")
            a_dist = 0.1
            T = 0.1

        start = [curpt.get_s(), curpt.get_v(), curpt.get_a()]
        end = [curpt.get_s() + a_dist, target_velocity, 0.0]

        #if T != acceltime:
        T = self.get_speedup_time(start, end)
        rospy.loginfo("acceltime was {:3.2f}, but T from get_speedup_time is {:3.2f}".format(acceltime, T))

        rospy.loginfo("Car set to accel from v={:3.3f}, a={:3.3f} to v={:3.3f}"
            " in dist {:3.3f} m in time {:3.3f} s"
            .format(curpt.get_v(), curpt.get_a(), target_velocity,
                    a_dist, T))

        jmt = JMT(start, end, T)
        self.JMT_List.append(jmt)
        jmt_ptr = len(self.JMT_List)
        return jmt_ptr-1

    def get_max_accel(self, ptr_id):
        # Use JMT to figure out shortest speedup distance using
        # max_desired_jerk after change in direction of acceleration
        curpt = self.waypoints[ptr_id]
        if (self.default_velocity - curpt.get_v()) < 0.2 and curpt.get_a() >= 0.0:
            rospy.logwarn("no big difference between velocities- just switch to desired at next wp")
            return (0.5, 0.5, 0.1)

        timer_start = rospy.get_time()
        too_short = False

        max_jerk = self.max_desired_jerk

        if curpt.get_a() < 0.0:
            accel_rate = 0.2
        else:  # assume here that this is startup condition
            accel_rate = self.default_accel

        a_dist = get_accel_distance(curpt.get_v(), self.default_velocity, accel_rate,
                                    curpt.get_a())
        T = get_accel_time(a_dist, curpt.get_v(), self.default_velocity)

        start = [curpt.get_s(), curpt.get_v(), curpt.get_a()]
        end = [curpt.get_s() + a_dist, self.default_velocity, 0.0]

        rospy.logdebug("Test accel_rate={:3.3f} from v={:3.3f}, a={:3.3f} to v={:3.3f}"
                       " in dist {:3.3f} m in time {:3.3f} s"
                       .format(accel_rate, curpt.get_v(), curpt.get_a(), self.default_velocity, a_dist, T))

        jmt = JMT(start, end, T)
        jerk = jmt.get_j_at(0.1)
        acc = jmt.get_a_at(T - 0.5)

        rospy.logdebug("Using accel = {:3.2f}, found initial jerk of {:3.2f} with a_dist of {:3.2f}"
                       " and final acc of {:3.2f} with a time of {:3.2f}"
                       .format(accel_rate, jerk, a_dist, acc ,T))

        if jerk > max_jerk:
            too_short = True
            acc_diff = -0.1
        else:
            acc_diff = 0.1
        if max_jerk == self.max_desired_jerk:
            acc_diff = acc_diff / 2.0

        optimized = False
        counter = 0
        while optimized is False:
            counter += 1

            old_jerk = jerk
            old_T = T
            old_dist = a_dist

            accel_rate = accel_rate + acc_diff
            if accel_rate < 0.0:
                final_accel = accel_rate - acc_diff
                duration = rospy.get_time() - timer_start
                rospy.logdebug("greatest accel_rate of {:3.2f} to accelerate with max_jerk"
                        "={:3.3f} from v={:3.3f}, a={:3.3f} to v={:3.3f} in "
                        "dist {:3.3f}m in time {:3.3f}s, took {:3.4f}s to calc"
                        .format(final_accel, jerk, curpt.get_v(), curpt.get_a(), self.default_velocity,
                                a_dist, T, duration))
                return(final_accel, a_dist, T)

            a_dist = get_accel_distance(curpt.get_v(), self.default_velocity, accel_rate,
                                    curpt.get_a())

            end = [curpt.JMTD.S + a_dist, self.default_velocity, 0.0]
            T = get_accel_time(a_dist, curpt.JMTD.V, self.default_velocity)

            rospy.logdebug("Test accel rate of {:3.2f} going from v={:3.3f}, a={:3.3f}"
                           " to v={:3.3f} in dist {:3.3f} m in time {:3.3f} s"
                           .format(accel_rate, curpt.get_v(), curpt.get_a(), self.default_velocity,
                                a_dist, T))

            jmt = JMT(start, end, T)
            jerk = jmt.get_j_at(0.1)
            acc = jmt.get_a_at(T - 0.5)

            rospy.logdebug("Using accel={:3.2f}, found initial jerk of {:3.2f} with a_dist of {:3.2f}"
                       " and final acc of {:3.2f} with a time of {:3.2f}"
                       .format(accel_rate, jerk, a_dist, acc ,T))

            if too_short is True:
                # looking for first instance that matches
                if jerk < max_jerk:
                    final_accel = accel_rate
                    optimized = True
            else:
                # looking for first instance that fails
                if jerk  > max_jerk:
                    final_accel = accel_rate - acc_diff
                    a_dist = old_dist
                    T = old_T
                    jerk = old_jerk
                    optimized = True

            if counter > 30:
                rospy.logwarn("counter is {} in get_max_accel - bail with acc={:3.2f}!"
                              .format(counter, accel_rate))
                final_accel = accel_rate
                optimized = True

        duration = rospy.get_time() - timer_start
        rospy.logdebug("get_max_accel found greatest accel_rate of {:3.2f} to accelerate with max_jerk"
                       "={:3.3f} from v={:3.3f}, a={:3.3f} to v={:3.3f} in "
                       "dist {:3.3f}m in time {:3.3f}s, took {:3.4f}s to calc"
                       .format(final_accel, jerk, curpt.get_v(), curpt.get_a(), self.default_velocity,
                               a_dist, T, duration))
        return (final_accel, a_dist, T)

    def get_speedup_time(self, start, end):
        # Use JMT to figure out proper time for acceleration where
        # curve does not wobbble above max _velocity
        # start[s, velocity, acc]
        # end[s, velocity, acc]
        save_start = copy.deepcopy(start)
        save_end = copy.deepcopy(end)
        if start[1] >= self.default_velocity * 0.95 and\
            start[2] >= 0.0:
            rospy.logwarn("Only small velocity change, time of 0.5 returned")
            return 0.5

        timer_start = rospy.get_time()

        if start[2] < 0.0:
            # currently slowing down
            time_factor = 1.0 #1.1
        else:
            time_factor = 1.0  # self.dyn_jmt_time_factor

        a_dist = end[0] - start[0]

        T = get_accel_time(a_dist, start[1], end[1]) * time_factor

        rospy.logdebug("Test accel from v={:3.3f}, a={:3.3f} to v={:3.3f}"
                       " in dist {:3.3f} m in time {:3.3f} s"
                       .format(start[1], start[2], end[1], a_dist, T))

        # time can not be shorter than this
        Tmin = (end[0] - start[0]) / self.default_velocity

        save_T = T
        try_reverse = False
        jmt = JMT(start, end, T)
        jerk = jmt.get_j_at(0.1)
        e_acc = jmt.get_a_at(T - 0.5)
        s_acc = jmt.get_a_at(0.5)

        rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and time of {:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        optimized = False
        time_diff = T * 0.04
        counter = 0
        while optimized is False:
            old_e_acc = e_acc
            old_s_acc = s_acc
            if e_acc > end[2] and s_acc > start[2]:
                optimized = True
            else:
                counter = counter + 1
                T = T + time_diff
                jmt = JMT(start, end, T)
                jerk = jmt.get_j_at(0.1)
                e_acc = jmt.get_a_at(T - 0.5)
                s_acc = jmt.get_a_at(0.5)
                rospy.logdebug("start2={:3.4f}s_acc={:3.4f}, end2={:3.4f}, e_acc={:3.4f}, s_acc-olds_acc={:3.4f}, s_acc-start2={:3.4f}, old_sacc-start2={:3.4f}, e_acc-olde_acc={:3.4f}, e_acc-end2={:3.4f}, old_eacc-end2={:3.4f}"
                        .format(start[2], s_acc, end[2], e_acc, s_acc-old_s_acc, s_acc-start[2], old_s_acc-start[2], e_acc-old_e_acc, e_acc-end[2], old_e_acc-end[2] ))
                if counter == 1 and\
                        (( e_acc < end[2] and math.fabs(e_acc - end[2]) > math.fabs(old_e_acc - end[2]) ) or
                        (s_acc < start[2] and math.fabs( start[2] - s_acc ) > math.fabs(start[2] - old_e_acc))):
                    rospy.loginfo("searching wrong direction - go the other way")
                    time_diff = 0.0 - time_diff

                if counter > 40 or T < Tmin:
                    rospy.logwarn("counter is {} in get_speedup_time - bail on searching in this direction!"
                                .format(counter))
                    optimized = True
                    try_reverse = True
                rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                            " using a_dist of {:3.2f}m and T={:3.2f}s"
                            .format(jerk, s_acc, e_acc, a_dist, T))
        if try_reverse == False:
            duration = rospy.get_time() - timer_start
            rospy.loginfo("get_speedup_time found shortest Time to accelerate with max_jerk={:3.3f}m/s^3 "
                      "from v={:3.3f}m/s, a={:3.3f}m/s^2 to v={:3.3f}m/s a={:3.3f}m/s^2 in dist {:3.3f}m"
                      " in time {:3.3f}s - Took {:3.4f}s to calc."
                      .format(jerk, start[1], start[2], end[1], end[2],
                              a_dist, T, duration))
            return T

        #########
        T = save_T
        start = save_start
        end = save_end
        jmt = JMT(start, end, T)
        jerk = jmt.get_j_at(0.1)
        e_acc = jmt.get_a_at(T - 0.5)
        s_acc = jmt.get_a_at(0.5)

        rospy.logdebug("Reverse found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                       " using a_dist of {:3.2f}m and time of {:3.2f}s"
                       .format(jerk, s_acc, e_acc, a_dist, T))

        optimized = False
        time_diff = 0.0 - time_diff

        counter = 0
        while optimized is False:
            old_e_acc = e_acc
            old_s_acc = s_acc
            if e_acc > end[2] and s_acc > start[2]:
                optimized = True
            else:
                counter = counter + 1
                T = T + time_diff
                jmt = JMT(start, end, T)
                jerk = jmt.get_j_at(0.1)
                e_acc = jmt.get_a_at(T - 0.5)
                s_acc = jmt.get_a_at(0.5)
                rospy.logdebug("start2={:3.4f}s_acc={:3.4f}, end2={:3.4f}, e_acc={:3.4f}, s_acc-olds_acc={:3.4f}, s_acc-start2={:3.4f}, old_sacc-start2={:3.4f}, e_acc-olde_acc={:3.4f}, e_acc-end2={:3.4f}, old_eacc-end2={:3.4f}"
                        .format(start[2], s_acc, end[2], e_acc, s_acc-old_s_acc, s_acc-start[2], old_s_acc-start[2], e_acc-old_e_acc, e_acc-end[2], old_e_acc-end[2] ))

                if counter > 30 or T < Tmin:
                    rospy.logwarn("counter is {} in get_speedup_time - bail!"
                                .format(counter))
                    optimized = True
                rospy.logdebug("found initial jerk of {:3.2f}m/s^3 and start_acc={:3.3f}m/s^2 end_acc={:3.3f}m/s^2"
                            " using a_dist of {:3.2f}m and T={:3.2f}s"
                            .format(jerk, s_acc, e_acc, a_dist, T))

        duration = rospy.get_time() - timer_start
        rospy.loginfo("get_speedup_time found shortest Time to accelerate with max_jerk={:3.3f}m/s^3 "
                      "from v={:3.3f}m/s, a={:3.3f}m/s^2 to v={:3.3f}m/s a={:3.3f}m/s^2 in dist {:3.3f}m"
                      " in time {:3.3f}s - Took {:3.4f}s to calc."
                      .format(jerk, start[1], start[2], end[1], end[2],
                              a_dist, T, duration))
        return T

    def check_point(self, pt):
        # check if JMT values at point exceed desired values
        recalc = False
        if pt.get_a() > self.max_accel:
            rospy.loginfo("A of {} exceeds max value of {} "
                          "at ptr = {}".format
                          (pt.get_a(), self.max_accel, pt.ptr_id))
            recalc = True
        if pt.get_j() > self.max_jerk:
            rospy.loginfo("J of {} exceeds max value of {} "
                          "at ptr = {}".format
                          (pt.get_j(), self.max_jerk, pt.ptr_id))
            recalc = True
        if pt.get_v() > self.max_velocity:
            rospy.loginfo("V of {} exceeds max value of {} "
                          "at ptr = {}".format
                          (pt.get_v(), self.max_velocity, pt.ptr_id))
            recalc = True
        return recalc

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def send_waypoints(self):
        # generates the list of LOOKAHEAD_WPS waypoints based on car location
        if self.dbw_enabled is False:
            self.reset_for_dbw()

        if self.got_to_end is False:
            self.final_waypoints_start_ptr = self.closest_waypoint()

        # do this at start of each cycle so that it doesn't change
        # if traffic cb happens in middle of loop
        if self.next_tl_wp_tmp >= self.final_waypoints_start_ptr:
            self.next_tl_wp = self.next_tl_wp_tmp
        else:
            self.next_tl_wp = -1

        # this manipulates self.next_tl_wp if self.testing is True
        if self.testing is True:
            self.run_tests()

        self.set_waypoints_velocity()

        lane = Lane()
        waypoints = []
        for wpt in self.waypoints[self.final_waypoints_start_ptr:
                                  self.final_waypoints_start_ptr +
                                  self.lookahead_wps]:
            waypoints.append(wpt.waypoint)

        lane.waypoints = list(waypoints)
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        self.pubs['/final_waypoints'].publish(lane)

        # Publish is_decelerating intention boolean for DBW node to use for
        # braking control
        if self.state is 'slowdown' or self.state is 'stopped':
            self.is_decelerating = True
        else:
            self.is_decelerating = False

        self.pubs['/is_decelerating'].publish(self.is_decelerating)

    def reset_for_dbw(self):
        self.got_to_end = False
        self.lookahead_wps = self.lookahead_wps_reset
        if self.velocity < self.min_moving_velocity:
            self.set_stopped(self.final_waypoints_start_ptr,
                             self.lookahead_wps)

    def closest_waypoint(self):

        found = False
        self.back_search = False

        def distance_lambda(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2)

        if self.waypoints and self.last_search_distance:
            dist = distance_lambda(
                self.waypoints[max(0,self.final_waypoints_start_ptr-1)].
                get_position(), self.pose.position)

            for i in range(self.final_waypoints_start_ptr, min(self.
                           final_waypoints_start_ptr + self.lookahead_wps, len(self.waypoints))):
                tmpdist = distance_lambda(self.waypoints[i].
                                          get_position(),
                                          self.pose.position)
                # rospy.logwarn("i={}, tmpdist={:3.2f}, dist={:3.2f},last_search={:3.2f}".format(i, tmpdist, dist, self.last_search_distance))
                if tmpdist < dist:
                    dist = tmpdist
                    if i == len(self.waypoints)-1:
                        # since shortest distance is at end of list, this is a match
                        if abs(dist - self.last_search_distance) < 5.0:
                            found = True
                            self.last_search_distance = dist
                            closest = i
                            break
                else:
                    # distance is starting to get larger so look at
                    # last position
                    if (i < self.final_waypoints_start_ptr + 1):
                        # we're closest to original waypoint, but what if
                        # we're going backwards - loop backwards to make sure
                        # a point further back  isn't closest
                        for j in range(max(i - 2,0),
                                       max(i - self.lookahead_wps - 1,0),
                                       -1):
                            tmpdist = distance_lambda(
                                self.waypoints[j % len(self.waypoints)].
                                get_position(),
                                self.pose.position)
                            # rospy.logwarn("j={}, tmpdist={:3.2f}, dist={:3.2f},last_search={:3.2f}".format(j, tmpdist, dist, self.last_search_distance))

                            if tmpdist <= dist:
                                dist = tmpdist
                                self.back_search = True
                            else:
                                if abs(dist-self.last_search_distance) < 5.0:
                                    self.last_search_distance = dist
                                    closest = ((j+1) % len(self.waypoints))
                                    found = True
                                    break
                                else:
                                    rospy.logwarn("j = {}, new dist {:4.3f} - "
                                                  "{:4.3f} > 5 "
                                                  .format(j, dist,
                                                  self.last_search_distance))
                                    break

                        # backwards local search was unsuccessful
                        # break so we don't fall back into forwards local
                        break

                    if abs(dist-self.last_search_distance) < 5.0:
                        found = True
                        self.last_search_distance = dist
                        closest = ((i - 1) %
                                len(self.waypoints))
                        break

                    rospy.logwarn("i = {}, new dist {:4.3f} - {:4.3f} > 5"
                                  .format(i, dist, self.last_search_distance))

            # fall out no closest match that looks acceptable
            if found is False:
                rospy.logwarn("waypoint_updater:closest_waypoint local search not "
                              "satisfied - run full search")

        x = self.pose.position.x
        y = self.pose.position.y

        if found is False:
            rospy.logdebug("waypoint_updater run KDTree full search")
            # run full search with KDTree
            closest = self.waypoint_tree.query([x,y],1)[1]
            self.last_search_distance = distance_lambda(
                                self.waypoints[closest].
                                get_position(),
                                self.pose.position)
            rospy.logdebug("KDTree: closest={}, dist={:3.2f}".format(closest, self.last_search_distance))

        # check to see if closest point is in front or behind car
        cl_vect = np.array(self.waypoints_2d[closest])
        prev_vect = np.array(self.waypoints_2d[closest-1])
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest = (closest + 1 ) % len(self.waypoints)
            self.last_search_distance = distance_lambda(
                                self.waypoints[closest].
                                get_position(),
                                self.pose.position)
            rospy.logdebug("Move ptr forward 1: closest={}, dist={:3.2f}"
                           .format(closest, self.last_search_distance))

        return closest


    def run_tests(self):
        self.next_tl_wp = -1
        ### Test for Track 2
        if self.final_waypoints_start_ptr in range(25,50):
            self.next_tl_wp = 50
            if self.state != 'slowdown' and self.final_waypoints_start_ptr >= 46:
                self.test_counter += 1
                if self.test_counter > 60:
                    self.next_tl_wp = -1

        ## Test for Track 1
        ## test of creep
        if self.final_waypoints_start_ptr in range(270,294):
            self.next_tl_wp = 293
            if self.state != 'creeping' and self.final_waypoints_start_ptr >= 288:
                self.test_counter += 1
                if self.test_counter > 60:
                    self.next_tl_wp = -1
        ## Test of seeing light too late
        if self.final_waypoints_start_ptr > 400 and self.final_waypoints_start_ptr < 411:
            self.next_tl_wp = 410
        if self.final_waypoints_start_ptr > 420 and self.final_waypoints_start_ptr < 480:
            self.test_counter = 0
            self.next_tl_wp = 500
        if self.final_waypoints_start_ptr >= 510 and self.final_waypoints_start_ptr < 753:
            self.next_tl_wp = 753
            if self.state != 'slowdown' and self.final_waypoints_start_ptr >= 748:
                self.test_counter += 1
                if self.test_counter > 60:
                    self.next_tl_wp = -1


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
