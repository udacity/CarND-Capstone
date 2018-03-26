#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight
from std_msgs.msg import Int32
import yaml
import time
from tf.transformations import euler_from_quaternion
from scipy.spatial.distance import euclidean
import math
from itertools import cycle, islice

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS   = 200   # Number of waypoints we will publish. For Test Lot, please put a value smaller than 60
CIRCULAR_WPS    = False # If True, assumes that the path to follow is a loop
REFRESH_RATE_HZ = 2     # Number of times we update the final waypoints per second
UPDATE_MAX_ITER = 50    # Max number of iterations before considering relooking for the next waypoint in full path
WAYPOINT_INCREMENT_RATE = 5 # The number of the waypoints that is added when searching for the next one ahead
DEBUG_MODE      = False # Switch for whether debug messages are printed.
TL_DETECTOR_ON  = False # If False, switches to direct traffic light subscription
DECELLERATION   = 3     # Decelleration in m/s^2


def normalize_angle(angle):
    if angle > math.pi:
        return angle - 2 * math.pi
    elif angle < -math.pi:
        return angle + 2 * math.pi
    else:
        return angle


def get_position(pos):
    return pos.position.x, pos.position.y, pos.position.z


def get_orientation(pos):
    return pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w


def euclidean_distance(wp1, wp2):
    return euclidean(get_position(wp1.pose.pose), get_position(wp2.pose.pose))


def step_by_step_distance(waypoints, wp1, wp2):
    dist = 0
    dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
    for i in range(wp1, wp2 + 1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG if DEBUG_MODE else rospy.WARN)

        if DEBUG_MODE:
            rospy.logwarn('waypoint_updater: DEBUG mode enabled - Disable for submission')

        if TL_DETECTOR_ON:
            rospy.logwarn('waypoint_updater: Traffic Light Detector disabled - Enable for submission')

        if WAYPOINT_INCREMENT_RATE != 1:
            rospy.logwarn('waypoint_updater: Waypoint increment rate should be set to 1 for submission')

        self.next_waypoint_indices = self.next_waypoint_indices_circular if CIRCULAR_WPS else self.next_waypoint_indices_simple

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        if TL_DETECTOR_ON:
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        else:
            config_string = rospy.get_param("/traffic_light_config")
            self.stop_line_config = yaml.load(config_string)
            self.stop_line_positions = self.stop_line_config['stop_line_positions']
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_array_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        self.next_waypoint = 0
        self.next_red_tl_wp = 0
        self.previous_pos = None
        self.static_waypoints = None
        self.static_velocities = None  # Static waypoints with adjusted velocity
        self.last_update = time.time()
        rospy.spin()

    def pose_cb(self, msg):
        self.previous_pos = msg.pose
        if time.time() - self.last_update > 1. / REFRESH_RATE_HZ:
            self.publish_update()

    def waypoints_cb(self, waypoints):
        self.static_waypoints = waypoints.waypoints  # Lane message is a structure with header and waypoints
        self.static_velocities = [self.get_static_waypoint_id_velocity(wp)
                                  for wp in range(len(self.static_waypoints))]
        rospy.loginfo('Received {} base waypoints'.format(len(self.static_waypoints)))

    def traffic_light_array_cb(self, msg):
        if self.static_waypoints and time.time() - self.last_update > 1. / REFRESH_RATE_HZ:
            tl_array = [l for l in msg.lights if self.waypoint_is_ahead(l)
                        and l.state != 2 and self.distance_to_previous(l.pose.pose) < 500]
            sorted_tl_array = sorted(tl_array, key=lambda x: self.distance_to_previous(x.pose.pose))

            # Only if we do have a red light not too far ahead, we publish a waypoint
            if len(sorted_tl_array):
                closest_tl = sorted_tl_array[0]
                tl_x, tl_y = closest_tl.pose.pose.position.x, closest_tl.pose.pose.position.y
                next_wps = self.next_waypoint_indices_simple()

                # Get corresponding stop line closest to traffic lght
                corresponding_stop_line = sorted(self.stop_line_positions,
                                          key=lambda x: euclidean(x, (tl_x, tl_y)))[0]

                # Get corresponding waypoint closest to stop line
                corresponding_wp = sorted(next_wps,
                                          key=lambda x: euclidean(corresponding_stop_line,
                                                                  (self.static_waypoints[x].pose.pose.position.x,
                                                                  self.static_waypoints[x].pose.pose.position.y)))[0]
                self.traffic_cb(Int32(corresponding_wp))
            else:
                self.traffic_cb(Int32(-1))

    def traffic_cb(self, msg):
        if self.next_red_tl_wp != msg.data:
            rospy.logdebug("===> Next Traffic Light: %i", msg.data)
            self.next_red_tl_wp = msg.data
            # 1: restore original speeds
            self.restore_all_velocities()
            # 2: gradually reduce speed in order to get to zero to next red light
            if self.next_red_tl_wp > 0:
                wp_indices = self.next_waypoint_indices()
                for wp in wp_indices:
                    tl_wp = self.static_waypoints[self.next_red_tl_wp]
                    check_wp = self.static_waypoints[wp]
                    if self.distance_to_previous(check_wp.pose.pose) > self.distance_to_previous(tl_wp.pose.pose) - 1:
                        self.set_waypoint_id_velocity(wp, 0.0)
                    else:
                        dist_wp_to_stop = euclidean_distance(check_wp, tl_wp)
                        current_wp_vel = self.get_static_waypoint_id_velocity(wp)
                        # Assuming car goes at 25 mph, i.e. 11.2 m/s, we need 30 meters to stop in order to ensure we
                        # stay below 5 m/s^2
                        target_vel = min(dist_wp_to_stop / (current_wp_vel / DECELLERATION), current_wp_vel)
                        self.set_waypoint_id_velocity(wp, target_vel)


    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_static_waypoint_id_velocity(self, waypoint_id):
        return self.static_waypoints[waypoint_id].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def set_waypoint_id_velocity(self, waypoint_id, velocity):
        self.static_waypoints[waypoint_id].twist.twist.linear.x = velocity

    def restore_waypoint_id_velocity(self, waypoint_id):
        self.set_waypoint_id_velocity(waypoint_id, self.static_velocities[waypoint_id])

    def restore_all_velocities(self):
        for wp_id in range(len(self.static_waypoints)):
            self.restore_waypoint_id_velocity(wp_id)

    def distance_to_previous(self, position):
        return euclidean(get_position(self.previous_pos), get_position(position))

    def waypoint_is_ahead(self, waypoint):
        w_x, w_y, _ = get_position(waypoint.pose.pose)
        p_x, p_y, _ = get_position(self.previous_pos)
        heading = math.atan2(w_y-p_y, w_x-p_x)
        yaw = euler_from_quaternion(get_orientation(self.previous_pos))[2]
        angle = normalize_angle(yaw-heading)
        return True if math.fabs(angle) < math.pi/4. else False

    def update_next_waypoint(self):
        it = 0
        nb_waypoints = len(self.static_waypoints)
        get_wp = lambda wp: wp % nb_waypoints if CIRCULAR_WPS else min(wp, nb_waypoints-1)
        while not self.waypoint_is_ahead(self.static_waypoints[get_wp(self.next_waypoint)]) and \
                it < UPDATE_MAX_ITER:
            it += 1
            self.next_waypoint = get_wp(self.next_waypoint+WAYPOINT_INCREMENT_RATE)  # We look at the next one

        # Searching the next waypoint in the full path takes much longer, we want to avoid it as much as possible
        if it == UPDATE_MAX_ITER:
            self.search_next_waypoint()

    def search_next_waypoint(self):
        self.next_waypoint = 0
        rospy.logwarn("Initiating search for closest waypoint...")
        # We basically search among all static waypoints the closest waypoint ahead
        for i in range(len(self.static_waypoints)):
            if self.waypoint_is_ahead(self.static_waypoints[i]) and \
                    self.distance_to_previous(self.static_waypoints[i].pose.pose) < \
                    self.distance_to_previous(self.static_waypoints[self.next_waypoint].pose.pose):
                self.next_waypoint = i
        rospy.logwarn('Found next closest waypoint: {}'.format(self.next_waypoint))

    # --- Next waypoint indices functions ---
    def next_waypoint_indices_circular(self):
        cyc = cycle(range(len(self.static_waypoints)))
        return list(islice(cyc, self.next_waypoint, self.next_waypoint + LOOKAHEAD_WPS))

    def next_waypoint_indices_simple(self):
        return list(islice(range(len(self.static_waypoints)), self.next_waypoint, self.next_waypoint+LOOKAHEAD_WPS))
    # ----------------------------------------

    def next_waypoints(self):
        indices = self.next_waypoint_indices()
        return [self.static_waypoints[i] for i in indices]

    def publish_update(self):
        # Emits the new waypoints, but only if we have received the base waypoints
        if self.static_waypoints:
            self.update_next_waypoint()
            rospy.logdebug("Next waypoint: {}".format(self.next_waypoint))
            self.final_waypoints_pub.publish(Lane(waypoints=self.next_waypoints()))

        self.last_update = time.time()


if __name__ == '__main__':
    print 'Starting waypoint_updater...'
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
