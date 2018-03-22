#!/usr/bin/env python

# File renamed to waypnt_updater.py because original name
# waypoint_updater.py which matches directory and package name
# prevented loading from waypoint_updater.cfg

import rospy
import math
from jmt import JMT, JMTDetails, JMTD_waypoint

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import String, Int32
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


def get_accel_distance(Vi, Vf, A):
    if A < 0.01:
        # avoid divide by 0 - should not happen
        return 50.0  # relatively long distance
    return math.fabs((Vf**2 - Vi**2)/(2.0 * A))


def get_accel_time(S, Vi, Vf):
    if (Vi + Vf) < 0.25:
        # avoid divide by 0
        return (0.0)
    return math.fabs((2.0 * S / (Vi + Vf)))


class WaypointUpdater(object):
    def __init__(self):
        self.dyn_vals_received = False
        self.waypoints = []
        self.pose = None
        self.velocity = None
        self.lights = None
        self.final_waypoints = []
        self.final_waypoints_start_ptr = 250 # 0
        self.back_search = False
        self.last_search_distance = None
        self.last_search_time = None
        self.next_tl_wp = None
        self.dyn_tl_buffer = 5.0  # tunable distance to stop before tl wp
        self.dyn_jmt_time_factor = 1.0  # tunable factor to make nicer s curve
        self.update_rate = 10
        self.max_velocity = 0.0
        self.max_accel = 5.0
        self.max_jerk = 5.0
        self.default_velocity = 10.7
        self.lookahead_wps = 100  # 200 is too many
        self.subs = {}
        self.pubs = {}
        self.dyn_reconf_srv = None
        self.max_s = 0.0  # length of track
        self.JMT_List = []
        # target max acceleration/braking force - dynamically adjustable
        self.default_accel = 1.5
        self.state = 'stopped'  # for now only use to see if stopped or moving

        rospy.init_node('waypoint_updater')

        self.subs['/base_waypoints'] = \
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.subs['/current_pose'] = \
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.subs['/current_velocity'] = \
            rospy.Subscriber('/current_velocity', TwistStamped,
                             self.velocity_cb)

        self.subs['/traffic_waypoint'] = \
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /obstacle_waypoint

        self.pubs['/final_waypoints'] = rospy.Publisher('/final_waypoints',
                                                        Lane, queue_size=1)

        # self.dyn_reconf_srv = Server(DynReconfConfig, self.dyn_vars_cb)
        # move to waypoints_cb so no collision - seems like a bad idea but
        # convenient for now

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

            if self.waypoints:
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
            old_test_stoplight_wp = self.next_tl_wp
            old_tl_buffer = self.dyn_tl_buffer
            old_jmt_time_factor = self.dyn_jmt_time_factor
        # end if

        rospy.loginfo("Received dynamic parameters {} with level: {}"
                      .format(config, level))

        if old_update_rate != config['dyn_update_rate']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting update Rate "
                          "from {} to {}".format(old_update_rate,
                                                 config['dyn_update_rate']))
            self.update_rate = config['dyn_update_rate']
            # need to switch the delay
            self.rate = rospy.Rate(self.update_rate)
        # end if

        if old_default_velocity != config['dyn_default_velocity']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting default_"
                          "velocity from {} to {}"
                          .format(old_default_velocity,
                                  config['dyn_default_velocity']))

            if config['dyn_default_velocity'] > self.max_velocity:
                rospy.logwarn("waypoint_updater:dyn_vars_cb default_velocity "
                              "limited to max_velocity {}"
                              .format(self.max_velocity))
                self.default_velocity = self.max_velocity * 0.975
                # config['dyn_default_velocity'] = self.default_velocity
            else:
                self.default_velocity = config['dyn_default_velocity']
            # end if
        # end if

        if old_default_accel != config['dyn_default_accel']:
            rospy.logwarn("waypoint_updater:dyn_vars_cb Adjusting default_"
                          "accel from {} to {}"
                          .format(old_default_accel,
                                  config['dyn_default_accel']))
            self.default_accel = config['dyn_default_accel']
        # end if

        if old_lookahead_wps != config['dyn_lookahead_wps']:
            rospy.loginfo("waypoint_updater:dyn_vars_cb Adjusting lookahead_"
                          "wps from {} to {}"
                          .format(old_lookahead_wps,
                                  config['dyn_lookahead_wps']))
            self.lookahead_wps = config['dyn_lookahead_wps']
        # end if

        if old_test_stoplight_wp != config['dyn_test_stoplight_wp']:
            rospy.logwarn("waypoint_updater:dyn_vars_cb Adjusting next "
                          "stoplight from {} to {}"
                          .format(old_test_stoplight_wp,
                                  config['dyn_test_stoplight_wp']))
            self.next_tl_wp = min(config['dyn_test_stoplight_wp'],
                                  len(self.waypoints)-1)
            # config['dyn_test_stoplight_wp'] = self.next_tl_wp
        # end if
        if old_tl_buffer != config['dyn_tl_buffer']:
            rospy.loginfo("dyn_vars_cb Adjusting tl_buffer"
                          "from {} to {}"
                          .format(old_tl_buffer,
                                  config['dyn_tl_buffer']))
            self.dyn_tl_buffer = config['dyn_tl_buffer']

        if old_jmt_time_factor != config['dyn_jmt_time_factor']:
            rospy.loginfo("dyn_vars_cb Adjusting jmt_time_factor"
                          "from {} to {}"
                          .format(old_tl_buffer,
                                  config['dyn_jmt_time_factor']))

            self.dyn_jmt_time_factor = config['dyn_jmt_time_factor']

        # we can also send adjusted values back
        return config

    def velocity_cb(self, twist_msg):
        # Check this is right
        self.velocity = twist_msg.twist.linear.x
        # TODO remove next line when verified correct
        rospy.loginfo("Velocity reported as {}".format(self.velocity))

    def pose_cb(self, pose_msg):
        # TODO refactor this to get position pose.pose.position
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
            wpt = None  # just to stop linter complaining

            for lanemsg_wpt in lane_msg.waypoints:

                if cntr > 0 and wpt:
                    # won't come into here until after wpt loaded
                    # in previous loop
                    s += math.sqrt((wpt.get_x() - lanemsg_wpt.pose.pose.
                                    position.x)**2 +
                                   (wpt.get_y() - lanemsg_wpt.pose.pose.
                                    position.y)**2)

                wpt = JMTD_waypoint(lanemsg_wpt, cntr, s)
                self.waypoints.append(wpt)

                if max_velocity < wpt.get_maxV():
                    max_velocity = wpt.get_maxV()
                # end if
                cntr += 1

            rospy.loginfo("waypoints_cb {} waypoints loaded, last waypoint "
                          "ptr_id = {} at s= {}".
                          format(len(self.waypoints), self.waypoints[cntr-1].
                                 ptr_id, self.waypoints[cntr-1].get_s()))
            self.max_s = self.waypoints[cntr-1].get_s()
            # setting max velocity based on project requirements in
            # Waypoint Updater Node Revisited
            self.max_velocity = max_velocity
            rospy.loginfo("waypoint_updater:waypoints_cb max_velocity set to "
                          " {} based on max value in waypoints."
                          .format(self.max_velocity))
            # now max_velocity is known, set up dynamic reconfig
            if not self.dyn_reconf_srv:
                self.dyn_reconf_srv = Server(DynReconfConfig, self.dyn_vars_cb)
                rospy.loginfo("dynamic_parm server started")
            # end if
        else:
            rospy.logerr("waypoint_updater:waypoints_cb attempt to load "
                         "waypoints when we have already loaded %d waypoints",
                         len(self.waypoints))
        # end if else
        self.subs['/base_waypoints'].unregister()
        rospy.loginfo("Unregistered from /base_waypoints topic")

    # Receive a msg from /traffic_waypoint about the next stop line
    def traffic_cb(self, traffic_msg):
        if traffic_msg.data != self.next_tl_wp:
            self.next_tl_wp = traffic_msg.data
            rospy.loginfo("new /traffic_waypoint message received at wp: %d."
                          "while car is at wp %d", self.next_tl_wp,
                          self.final_waypoints_start_ptr)
        else:
            # just for debug to see what we're getting
            rospy.loginfo("same /traffic_waypoint message received.")

    def get_dist_to_tl(self):
        # this can happen before we get a traffic_wp msg
        if not self.next_tl_wp:
            self.next_tl_wp = -1
        if self.next_tl_wp > -1:  # self.final_waypoints_start_ptr:
            # TODO does not account for looping - don't think it needs to
            dist_to_tl = math.fabs(self.waypoints[self.next_tl_wp - 1].get_s() -
                                   self.waypoints[self.final_waypoints_start_ptr]
                                   .get_s())
        else:
            dist_to_tl = 5000  # big number
        return dist_to_tl

    def set_stopped(self, start_ptr, num_wps):
        # set V = 0 for waypoints in range
        for ptr in range(start_ptr, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            self.waypoints[mod_ptr].JMTD.set_VAJt(0.0, 0.0, 0.0, 0.0)
            self.waypoints[mod_ptr].set_v(0.0)
            self.waypoints[mod_ptr].JMT_ptr = -1

    def set_transition_to_stop(self, mod_ptr):
        # gracefully slow down to stopped at end of jmt decel curve
        disp = self.waypoints[mod_ptr].get_s() -\
            self.waypoints[mod_ptr-1].get_s()

        decel = self.waypoints[mod_ptr-1].JMTD.A * 0.5

        velocity = min(max(0.0, self.waypoints[mod_ptr-1].get_v() -
                           math.sqrt(math.fabs(decel) * disp * 2.0)),
                       self.waypoints[mod_ptr].get_maxV())
        if velocity < 0.1:
            velocity = 0.0
            decel = 0.0
        # rospy.loginfo("velocity set to {} using accel = {} "
        #             "and disp = {} at ptr = {}"
        #             .format(velocity, decel, disp, mod_ptr))
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
        rospy.loginfo("velocity set to {} using accel = {} "
                      "and disp = {} at ptr = {}"
                      .format(jmt_pnt.V, jmt_pnt.A, pt.get_s(), pt.ptr_id))
        return jmt_pnt.time

    def produce_slowdown(self, start_ptr, num_wps, safety_factor):
        # this generates the deceleration curve based on JMT
        # for the range of waypoints, inserting V = 0.0
        # beyond the stop point
        recalc = False
        curpt = self.waypoints[start_ptr]
        target_velocity = 0.0

        if curpt.JMT_ptr == -1 or self.state != 'slowdown':
            jmt_ptr = self.setup_jmt(curpt, target_velocity, safety_factor,
                                     self.dyn_jmt_time_factor)
            curpt.JMT_ptr = jmt_ptr
            self.state = 'slowdown'
        else:
            jmt_ptr = curpt.JMT_ptr
            rospy.loginfo("using old jmt_ptr = {}".format(jmt_ptr))
        JMT_instance = self.JMT_List[jmt_ptr]

        t = 0.0
        for ptr in range(start_ptr + 1, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            if curpt.get_s() <= JMT_instance.final_displacement:
                # create the main part of the jmt curve
                t = self.gen_point_in_jmt_curve(curpt, jmt_ptr, t)
                if self.check_point(curpt) is True:
                    recalc = True
            else:
                # the car has reached the point where the JMT
                # curve should reach target velocity of 0.0
                # we smooth the transition
                rospy.loginfo("{} beyond S = {} at ptr_id = {}".
                              format(curpt.get_s(), JMT_instance.
                                     final_displacement, mod_ptr))
                self.set_transition_to_stop(mod_ptr)
        return recalc

    def maintain_speed(self, start_ptr, num_wps):
        # sets the speed of the car to default velocity for the
        # range of waypoints
        for ptr in range(start_ptr, start_ptr + num_wps):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            rospy.loginfo("setting to default velocity {} at ptr = {}"
                          .format(self.default_velocity, mod_ptr))

            velocity = min(curpt.get_maxV(), self.default_velocity)
            curpt.set_v(velocity)
            curpt.JMTD.set_VAJt(velocity, 0.0, 0.0, 0.0)
            curpt.JMT_ptr = -1

    def init_acceleration(self, start_ptr, num_wps):
        # starts the car moving with a lead foot
        offset = 0

        velocity = max(0.4, self.waypoints[start_ptr].get_v())
        self.waypoints[start_ptr].set_v(velocity)
        self.waypoints[start_ptr].JMTD.set_VAJt(
                    velocity, self.default_accel, 0.0, 0.0)
        offset += 1
        while velocity < 1.5 and offset < num_wps:
            disp = self.waypoints[start_ptr + offset].get_s() -\
                self.waypoints[start_ptr].get_s()
            velocity = min(max(0.5, math.sqrt(self.default_accel * disp * 2.0)),
                           self.waypoints[start_ptr + offset].get_maxV())
            rospy.loginfo("velocity set to {} using accel = {} and "
                          "disp = {} at ptr = {}"
                          .format(velocity, self.default_accel, disp,
                                  start_ptr + offset))
            self.waypoints[start_ptr + offset].set_v(velocity)
            self.waypoints[start_ptr + offset].JMTD.set_VAJt(
                velocity, self.default_accel, 0.0, 0.0)
            self.waypoints[start_ptr + offset].JMT_ptr = -1
            offset += 1
        return offset - 1

    def generate_speedup(self, start_ptr, end_ptr):
        # this is triggered after initiate_aceleration()
        # gets the car moving - that way we can adjust them
        # independently
        recalc = False
        curpt = self.waypoints[start_ptr]

        if curpt.JMT_ptr == -1 or self.state != 'speedup':
            self.state = 'speedup'
            curpt.JMT_ptr = self.setup_jmt(curpt, self.default_velocity)
            # smooth_factor = 1.0, time_factor = 1.0)
            jmt_ptr = curpt.JMT_ptr
        else:
            jmt_ptr = curpt.JMT_ptr
            rospy.loginfo("using old jmt_ptr = {}".format(curpt.JMT_ptr))
        JMT_instance = self.JMT_List[jmt_ptr]

        t = 0.0
        for ptr in range(start_ptr+1, end_ptr):
            mod_ptr = ptr % len(self.waypoints)
            curpt = self.waypoints[mod_ptr]

            if curpt.get_s() <= JMT_instance.final_displacement:
                # create the main part of the jmt curve
                t = self.gen_point_in_jmt_curve(curpt, jmt_ptr, t)
                if self.check_point(curpt) is True:
                    recalc = True
            else:
                # The car has reached the point where it is near to target
                # velocity
                rospy.loginfo("{} beyond S = {} at ptr_id = {}".format
                              (curpt.get_s(), JMT_instance.final_displacement,
                               mod_ptr))
                self.maintain_speed(mod_ptr, 1)
        return recalc

    def set_waypoints_velocity(self):
        offset = 0           # offset in front of car to account for latency
        safety_factor = 2.0  # increase slowing down distance
        recalc = False       # indication that JMT calcs exceed bounds

        dist_to_tl = self.get_dist_to_tl()

        if self.waypoints[self.final_waypoints_start_ptr].get_v() == 0.0:
            # we are stopped
            offset = 0
            stopping_distance = 0.0
            self.state = 'stopped'
        else:
            stopping_distance = get_accel_distance(
                self.waypoints[self.final_waypoints_start_ptr].get_v(),
                0.0, self.default_accel/safety_factor)

        rospy.loginfo("dist_to_tl = {}, stopping_dist = {}, state = {}"
                      .format(dist_to_tl, stopping_distance, self.state))

        # handle case where car is stopped at lights and light is red
        if self.state == 'stopped' and dist_to_tl < self.dyn_tl_buffer:
            self.set_stopped(self.final_waypoints_start_ptr, self.
                             final_waypoints_start_ptr + self.lookahead_wps)

        elif dist_to_tl < stopping_distance + self.dyn_tl_buffer or\
                dist_to_tl < 20:    # added last check in to try to prevent
                                    # flipping between accel and decel
                                    # when approaching the light

            if dist_to_tl < self.dyn_tl_buffer:
                # small buffer from stop line - stop the car if in this area
                rospy.loginfo("Now within tl_buffer = {}".format(self.dyn_tl_buffer))
                self.set_stopped(self.final_waypoints_start_ptr, self.
                                 lookahead_wps)
            else:
                # reduce car velocity to 0.0 stoping before lights
                recalc = self.produce_slowdown(self.final_waypoints_start_ptr,
                                               self.lookahead_wps,
                                               safety_factor)
            # end if else
        elif self.waypoints[self.final_waypoints_start_ptr].get_v() >= \
                self.default_velocity:
                # handle case where car is at target speed and no
                # traffic lights within stopping distance
            self.maintain_speed(self.final_waypoints_start_ptr, self.
                                lookahead_wps)
        else:
            if self.waypoints[self.final_waypoints_start_ptr].get_v() < 1.5:
                # Get the car going before handing off to JMT
                offset =\
                    self.init_acceleration(self.final_waypoints_start_ptr,
                                           self.lookahead_wps)

            recalc = self.generate_speedup(self.final_waypoints_start_ptr + offset,
                                           self.final_waypoints_start_ptr +
                                           self.lookahead_wps)
        if recalc is True:
            rospy.logwarn("recalc is set to {} we should recalculate"
                          .format(recalc))

        rospy.loginfo("ptr_id, JMT_ptr, S, V, A, J, time")
        for wpt in self.waypoints[self.final_waypoints_start_ptr:
                                  self.final_waypoints_start_ptr +
                                  self.lookahead_wps]:
            rospy.loginfo("{}, {}, {}".format(wpt.ptr_id, wpt.JMT_ptr, wpt.JMTD))

    def setup_jmt(self, curpt, target_velocity, smooth_factor=1.0, 
                  time_factor=1.0):
        # How can we adjust these to get a smoother curve

        # smooth_factor set > 1.0 reduces the peak accel rates produced
        # by extending the distance to slow down
        a_dist = get_accel_distance(curpt.JMTD.V, target_velocity,
                                    self.default_accel/smooth_factor)
        # time_factor values set > 1 stretch out the start or end of the curve
        # where the car is going slowest
        T = get_accel_time(a_dist, curpt.JMTD.V, target_velocity) * time_factor

        if a_dist < 0.1 or T < 0.1:
            # dummy values to prevent matrix singularity
            # if no velocity change required
            a_dist = 0.1
            T = 0.1

        rospy.loginfo("Setup JMT to accelerate to {} in dist {} m in time {} s"
                      .format(target_velocity, a_dist, T))

        start = [curpt.JMTD.S, curpt.JMTD.V, curpt.JMTD.A]
        end = [curpt.JMTD.S + a_dist, target_velocity, 0.0]
        jmt = JMT(start, end, T)
        self.JMT_List.append(jmt)
        jmt_ptr = len(self.JMT_List)

        return jmt_ptr-1

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
        # for now assume waypoints form a loop - may not be the case

        self.final_waypoints_start_ptr = self.closest_waypoint()
        # end_wps_ptr = (self.final_waypoints_start_ptr +
        #                self.lookahead_wps) % len(self.waypoints)
        # rospy.loginfo("waypoint_updater:send_waypoints start_wps_ptr = %d,"
        #               " end_wps_ptr = %d", self.final_waypoints_start_ptr,
        #               end_wps_ptr)
        # if end_wps_ptr > self.final_waypoints_start_ptr:
        #     for w_p in self.waypoints[self.final_waypoints_start_ptr:
        #                               end_wps_ptr]:
        #         new_wps_list.append(w_p)
        #     # end of for
        # else:
        #     for w_p in self.waypoints[self.final_waypoints_start_ptr:]:
        #         new_wps_list.append(w_p)
        #     # end of for
        #     for w_p in self.waypoints[:end_wps_ptr]:
        #         new_wps_list.append(w_p)
        #     # end of for
        # # end of if

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

    def closest_waypoint(self):
        # TODO - use local search first of final_waypoints sent out last
        # iteration
        def distance_lambda(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2)
        # TODO: move away from using final waypoint, just use waypoints
        # since we have saved original v info within the structure
        if self.waypoints and self.last_search_distance:
            dist = distance_lambda(
                self.waypoints[self.final_waypoints_start_ptr-1].get_position(),
                                   self.pose.position)
            for i in range(self.final_waypoints_start_ptr, self.final_waypoints_start_ptr + self.lookahead_wps):
                tmpdist = distance_lambda(self.waypoints[i].
                                          get_position(),
                                          self.pose.position)
                if tmpdist < dist:
                    dist = tmpdist
                else:
                    # distance is starting to get larger so look at
                    # last position
                    if (i <= self.final_waypoints_start_ptr + 1):
                        # we're closest to original waypoint, but what if
                        # we're going backwards - loop backwards to make sure
                        # a point further back  isn't closest
                        for j in range(i - 1,
                                       i - self.lookahead_wps,
                                       -1):
                            tmpdist = distance_lambda(
                                self.waypoints[j % len(self.waypoints)].
                                get_position(),
                                self.pose.position)
                            if tmpdist < dist:
                                dist = tmpdist
                                self.back_search = True
                            else:
                                if abs(dist-self.last_search_distance) < 5.0:
                                    self.last_search_distance = dist
                                    return ((j+1) % len(self.waypoints))
                                else:
                                    rospy.logwarn("j = {}, new dist {} - {} > 5 ".format(j, dist, self.last_search_distance))
                                    break
                            # end if else
                        # end for
                    # end if
                    if abs(dist-self.last_search_distance) < 5.0:
                        self.last_search_distance = dist
                        return ((i - 1) %
                                len(self.waypoints))
                    rospy.logwarn("i = {}, new dist {} - {} > 5".format(i, dist, self.last_search_distance))
                    # end if
                # end if else
            # end for - fall out no closest match that looks acceptable
            rospy.logwarn("waypoint_updater:closest_waypoint local search not"
                          "satisfied - run full search")
        # end if

        dist = 1000000  # maybe should use max
        closest = 0
        for i in range(len(self.waypoints)):
            tmpdist = distance_lambda(self.waypoints[i].get_position(),
                                      self.pose.position)
            if tmpdist < dist:
                closest = i
                dist = tmpdist
            # end of if
        # end of for
        self.last_search_distance = dist
        return closest
        # Note: the first waypoint is closest to the car, not necessarily in
        # front of it.  Waypoint follower is responsible for finding this

    # Todo need to adjust for looping
    def distance(self, wp1, wp2):
        dist = 0

        def distance_lambda(a, b): return math.sqrt((a.x-b.x)**2 +
                                                    (a.y-b.y)**2 +
                                                    (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += distance_lambda(self.waypoints[wp1].get_position(),
                                    self.waypoints[i].get_position())
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
