#!/usr/bin/env python
# File renamed to waypnt_updater.py because original name
# waypoint_updater.py which matches directory and package name
# prevented loading from waypoint_updater.cfg

import rospy
import math
import copy

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


class WaypointUpdater(object):
    def __init__(self):
        self.dyn_vals_received = False
        self.waypoints = []
        self.pose = None
        self.velocity = None
        self.lights = None
        self.final_waypoints = []
        self.final_waypoints_start_ptr = 0
        self.back_search = False
        self.last_search_distance = None
        self.last_search_time = None
        self.next_traffic_light_wp = None
        self.update_rate = 10
        self.max_velocity = 0.0
        self.default_velocity = 10.0
        self.lookahead_wps = 200
        self.subs = {}
        self.pubs = {}
        self.dyn_reconf_srv = None

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
            old_lookahead_wps = self.lookahead_wps
            old_test_stoplight_wp = self.next_traffic_light_wp
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
                self.default_velocity = self.max_velocity
            else:
                self.default_velocity = config['dyn_default_velocity']
            # end if
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
            self.next_traffic_light_wp = config['dyn_test_stoplight_wp']
        # end if

        # we can also send adjusted values back
        return config

    def velocity_cb(self, twist_msg):
        self.velocity = twist_msg

    def pose_cb(self, pose_msg):
        self.pose = pose_msg.pose
        rospy.logdebug("waypoint_updater:pose_cb pose set to  %s", self.pose)

    # Load set of waypoints from /basewaypoints into self.waypoints
    # this should only happen once, so we unsubscribe at end
    def waypoints_cb(self, lane_msg):
        rospy.loginfo("waypoint_updater:waypoints_cb loading waypoints")
        if not self.waypoints:
            max_velocity = 0.0
            for waypoint in lane_msg.waypoints:
                self.waypoints.append(waypoint)
                if max_velocity < self.get_waypoint_velocity(waypoint):
                    max_velocity = self.get_waypoint_velocity(waypoint)
                # end if
            rospy.loginfo("waypoint_updater:waypoints_cb %d waypoints loaded",
                          len(self.waypoints))
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
        if traffic_msg.data != self.next_traffic_light_wp:
            # do we need to queue these or will I only get the next one
            # coming up?
            # If car is stopped do I need to ramp up velocity, then
            # down to reach it?
            self.next_traffic_light_wp = traffic_msg.data
            rospy.loginfo("new /traffic_waypoint message received at wp: %d."
                          "while car is at wp %d", self.next_traffic_light_wp,
                          self.final_waypoints_start_ptr)
        else:
            # just for debug to see what we're getting
            rospy.loginfo("same /traffic_waypoint message received.")

    # adjust the velocities in the /final_waypoint queue
    # TODO: move the waypoints to a structure there first so don't have to
    # worry about looping
    def set_waypoints_velocity(self):
        # this can happen before we get a traffic_wp msg
        if not self.next_traffic_light_wp:
            # set one up behind car - is this a problem if we reverse?
            # STUB in putting it at 759 to see if it works
            # next_traffic_light_wp = self.final_waypoints_start_ptr - 1
            self.next_traffic_light_wp = 759
            next_traffic_light_wp = self.next_traffic_light_wp
        else:
            next_traffic_light_wp = self.next_traffic_light_wp
        # end if else
        if ((next_traffic_light_wp - self.final_waypoints_start_ptr) %
                len(self.waypoints)) < 100:
            start_ptr = self.final_waypoints_start_ptr + 5
            end_ptr = next_traffic_light_wp
            distance_to_rl = self.distance(start_ptr, end_ptr)
            velocity = self.waypoints[start_ptr].twist.twist.linear.x
            # time_to_decel = 2 * distance_to_rl / velocity
            # accel = - velocity / time_to_decel
            chord_dist = 0
            final_end_ptr = self.final_waypoints_start_ptr + self.lookahead_wps
            if distance_to_rl < 1.0:  # small buffer from stop line
                for ptr in range(start_ptr, final_end_ptr):
                    mod_ptr = ptr % len(self.waypoints)
                    self.waypoints[mod_ptr].twist.twist.linear.x = 0.0
                # end for
            else:
                for ptr in range(start_ptr, final_end_ptr):
                    # self.next_traffic_light_wp):
                    mod_ptr = ptr % len(self.waypoints)
                    chord_dist += self.distance(ptr, ptr+1)
                    v = (1-chord_dist/distance_to_rl)*velocity
                    if v < 0.9:
                        v = 0.0
                    self.waypoints[mod_ptr].twist.twist.linear.x = v
                # end for
            # end if else
        else:
            # if self.velocity < self.default_velocity:
            #    waypoints[self.final_waypoints_start_ptr + 5].twist.twist.\
            #        linear.x == self.velocity:
            start_ptr = self.final_waypoints_start_ptr
            final_end_ptr = self.final_waypoints_start_ptr + self.lookahead_wps
            for ptr in range(start_ptr, final_end_ptr):
                mod_ptr = ptr % len(self.waypoints)
                self.waypoints[mod_ptr].twist.twist.linear.x = \
                    self.default_velocity
                # end for
            # end if
        # maybe this should be called from main loop

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def send_waypoints(self):
        # generates the list of LOOKAHEAD_WPS waypoints based on car location
        # for now assume waypoints form a loop
        new_wps_list = []
        start_wps_ptr = self.closest_waypoint()
        end_wps_ptr = (start_wps_ptr +
                       self.lookahead_wps) % len(self.waypoints)
        rospy.loginfo("waypoint_updater:send_waypoints start_wps_ptr = %d,"
                      " end_wps_ptr = %d", start_wps_ptr, end_wps_ptr)
        if end_wps_ptr > start_wps_ptr:
            for w_p in self.waypoints[start_wps_ptr:end_wps_ptr]:
                # w_p.twist.twist.linear.x = self.default_velocity
                new_wps_list.append(w_p)
            # end of for
        else:
            for w_p in self.waypoints[start_wps_ptr:]:
                # w_p.twist.twist.linear.x = self.default_velocity
                new_wps_list.append(w_p)
            # end of for
            for w_p in self.waypoints[:end_wps_ptr]:
                # w_p.twist.twist.linear.x = self.default_velocity
                new_wps_list.append(w_p)
            # end of for
        # end of if
        # for now this should work, as no deepcopy. lets see
        self.set_waypoints_velocity()
        rospy.loginfo("waypoint_updater:send_waypoints final_waypoints list"
                      " length is %d", len(new_wps_list))
        lane = Lane()
        lane.waypoints = list(new_wps_list)
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time()
        self.pubs['/final_waypoints'].publish(lane)
        self.final_waypoints = list(new_wps_list)
        self.final_waypoints_start_ptr = start_wps_ptr

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypointlist, waypoint, velocity):
        waypointlist[waypoint].twist.twist.linear.x = velocity

    def closest_waypoint(self):
        # TODO - use local search first of final_waypoints sent out last
        # iteration
        def distance_lambda(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2)
        if self.final_waypoints:
            dist = distance_lambda(self.final_waypoints[0].pose.pose.position,
                                   self.pose.position)
            for i in range(1, len(self.final_waypoints)):
                tmpdist = distance_lambda(self.final_waypoints[i].pose.pose.
                                          position, self.pose.position)
                if tmpdist < dist:
                    dist = tmpdist
                else:
                    # distance is starting to get larger so look at
                    # last position
                    if (i == 1):
                        # we're closest to original waypoint, but what if
                        # we're going backwards - loop backwards to make sure
                        # a point further back  isn't closest
                        for j in range(self.final_waypoints_start_ptr-1,
                                       self.final_waypoints_start_ptr -
                                       len(self.final_waypoints),
                                       -1):
                            tmpdist = distance_lambda(
                                self.waypoints[j % len(self.waypoints)].
                                pose.pose.position,
                                self.pose.position)
                            if tmpdist < dist:
                                dist = tmpdist
                                self.back_search = True
                            else:
                                if abs(dist-self.last_search_distance) < 5.0:
                                    self.last_search_distance = dist
                                    return ((j+1) % len(self.waypoints))
                                else:
                                    break
                            # end if else
                        # end for
                    # end if

                    if abs(dist-self.last_search_distance) < 5.0:
                        self.last_search_distance = dist
                        return ((self.final_waypoints_start_ptr + i - 1) %
                                len(self.waypoints))
                    # end if
                # end if else
            # end for - fall out no closest match that looks acceptable
            rospy.logwarn("waypoint_updater:closest_waypoint local search not"
                          "satisfied - run full search")
        # end if

        dist = 1000000  # maybe should use max
        closest = 0
        for i in range(len(self.waypoints)):
            tmpdist = distance_lambda(self.waypoints[i].pose.pose.position,
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
            dist += distance_lambda(self.waypoints[wp1].pose.pose.position,
                                    self.waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
