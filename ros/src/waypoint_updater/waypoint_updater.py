#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import time
from tf.transformations import euler_from_quaternion
from scipy.spatial.distance import euclidean
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this
REFRESH_RATE_HZ = 5 # Number of times we update the final wyapoints per second
UPDATE_MAX_ITER = 50 # Max number of iterations before considering relooking for the next waypoint in full path

def get_position(pos):
    return pos.position.x, pos.position.y, pos.position.z

def get_orientation(pos):
    return pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.next_waypoint = 0
        self.previous_pos = None
        self.static_waypoints = None
        self.last_update = time.time()
        rospy.spin()

    def pose_cb(self, msg):
        self.previous_pos = msg.pose
        if time.time() - self.last_update > 1. / REFRESH_RATE_HZ:
            self.publish_update()

    def waypoints_cb(self, waypoints):
        self.static_waypoints = waypoints  # Lane message is a structure with header and waypoints
        rospy.loginfo('Received {} base waypoints'.format(len(self.static_waypoints.waypoints)))


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

    def distance_to_previous(self, position):
        return euclidean(get_position(self.previous_pos), get_position(position))

    def waypoint_is_ahead(self, waypoint):
        w_x, w_y, _ = get_position(waypoint.pose.pose)
        p_x, p_y, _ = get_position(self.previous_pos)
        heading = math.atan2(w_y-p_y, w_x-p_x)
        yaw = euler_from_quaternion(get_orientation(self.previous_pos))[2]
        angle = yaw-heading
        return True if math.fabs(angle) < math.pi/4. else False

    def update_next_waypoint(self):
        it = 0
        nb_waypoints = len(self.static_waypoints.waypoints)
        while not self.waypoint_is_ahead(self.static_waypoints.waypoints[self.next_waypoint % nb_waypoints]) and \
                it < UPDATE_MAX_ITER:
            self.next_waypoint += 1  # We look at the next one

        # Searching the next waypoint in the full path takes much longer, we want to avoid it as much as possible
        if it == UPDATE_MAX_ITER:
            self.search_next_waypoint()

    def search_next_waypoint(self):
        self.next_waypoint = 0
        # TODO: Rewrite this in a clearer way
        # We basically search among all static waypoints the closest waypoint ahead
        for i in range(len(self.static_waypoints)):
            if self.waypoint_is_ahead(self.static_waypoints.waypoints[i]) and \
                    self.distance_to_previous(self.static_waypoints.waypoints[i].pose) < \
                    self.distance_to_previous(self.static_waypoints.waypoints[self.next_waypoint].pose):
                self.next_waypoint = i
        rospy.loginfo('Found next waypoint: {}'.format(self.next_waypoint))

    def publish_update(self):
        # Emits the new waypoints, but only if we have received the base waypoints
        if self.static_waypoints:
            self.update_next_waypoint()
            wp = self.next_waypoint
            self.final_waypoints_pub.publish(Lane(waypoints=self.static_waypoints.waypoints[wp:wp+LOOKAHEAD_WPS]))

        self.last_update = time.time()


if __name__ == '__main__':
    print 'Starting waypoint_updater...'
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
