#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from visualization_msgs.msg import Marker, MarkerArray

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


def waypointToMarker(waypoint, frame_id, ts=rospy.Time(0), idx=0, color=[0.0, 1.0, 0.0]):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ts
    marker.id = idx
    marker.pose.position = waypoint.pose.pose.position
    marker.pose.position.z = 2  # show above other markers...
    marker.pose.orientation = waypoint.pose.pose.orientation
    marker.scale.x = 5
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.get_all_waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_waypoints_marker_pub = rospy.Publisher('final_waypoints_markers', MarkerArray, queue_size=1)

        # Current state
        self.car_pose_x = None
        self.car_pose_y = None
        self.waypoints = None
        self.nearest_idx = None

        rospy.spin()

    def pose_cb(self, msg):
        """Update the next waypoints using the current car pose."""
        self.car_pose_x = msg.pose.position.x
        self.car_pose_y = msg.pose.position.y

        if not self.waypoints:
            return  # not yet ready

        self.update_nearest_waypoint()
        self.set_velocities()
        self.publish_final_waypoints()

    def set_velocities(self):
        """Set the velocities of the next waypoints."""
        for i in range(LOOKAHEAD_WPS):
            wp = self.nearest(offset=i)
            wp.twist.twist.linear.x = 10  # TODO: set linear velocity to something sensible

    def publish_final_waypoints(self, visualize=True):
        """Publish the next waypoints."""
        lane = Lane()
        lane.header.frame_id = 'world'
        lane.header.stamp = rospy.Time.now()
        for i in range(LOOKAHEAD_WPS):
            wp = self.nearest(offset=i)
            lane.waypoints.append(wp)

        self.final_waypoints_pub.publish(lane)
        if visualize:
            self.publish_markers()

    def publish_markers(self):
        """Publish markers for the next waypoints."""
        green = [0.0, 1.0, 0.0]
        yellow = [1.0, 1.0, 0.0]
        array = MarkerArray()

        for i in range(LOOKAHEAD_WPS):
            if i % 10 == 0:  # only publish every tenth waypoint as a marker
                array.markers.append(
                    waypointToMarker(self.nearest(i), 'world', ts=rospy.Time.now(), idx=i, color=yellow))

        self.final_waypoints_marker_pub.publish(array)

    def nearest(self, offset=0):
        """Get current nearest waypoint."""
        return self.waypoints[self.nearest_idx + offset]

    def update_nearest_waypoint(self):
        """Find the index of the currently nearest waypoint."""
        if not self.nearest_idx or self.dist_to(self.nearest()) > 10.0:
            nearest_dist = float('inf')
            nearest_idx = 0
            for i, waypoint in enumerate(self.waypoints):
                if self.dist_to(waypoint) < nearest_dist:
                    nearest_dist = self.dist_to(waypoint)
                    nearest_idx = i
            self.nearest_idx = nearest_idx
            rospy.loginfo("Nearest waypoint is at index {} with distance {} m.".format(nearest_idx, nearest_dist))
        else:
            # begin check from current index TODO: Handle index wrap around (e.g. after one full round driven).
            nearest_idx = self.nearest_idx
            nearest_dist = self.dist_to(self.waypoints[nearest_idx])

            for i in range(100):
                current_idx = nearest_idx + i
                current_waypoint = self.waypoints[current_idx]
                if self.dist_to(current_waypoint) < nearest_dist:
                    nearest_idx = current_idx
                    nearest_dist = self.dist_to(current_waypoint)
            self.nearest_idx = nearest_idx
        # rospy.logdebug("Nearest waypoint: {}, dist: {}".format(self.nearest_waypoint_idx, self.dist_to(self.nearest())))

    def dist_to(self, waypoint):
        """Compute distance from current car position to a waypoint."""
        wp = waypoint.pose.pose.position
        return math.sqrt((self.car_pose_x - wp.x) ** 2 + (self.car_pose_y - wp.y) ** 2)

    def get_all_waypoints_cb(self, lane):
        """Subscribe to the list of all waypoints."""
        self.waypoints = lane.waypoints
        rospy.loginfo("Received waypoints...")

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
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
