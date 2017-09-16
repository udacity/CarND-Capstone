#!/usr/bin/env python
"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
"""

from geometry_msgs.msg import PoseStamped
import math
import rospy
from styx_msgs.msg import Lane, Waypoint


def get_waypoint_velocity(waypoint):
    """Get the velocity from a given waypoint."""
    return waypoint.twist.twist.linear.x


def set_waypoint_velocity(waypoints, index, velocity):
    """Set the velocity of the waypoint at the given index."""
    waypoints[index].twist.twist.linear.x = velocity


def distance(waypoints, index_1, index_2):
    """Calculate the distance between two waypoints using a piece-wise linear function."""
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
    for i in range(index_1, index_2+1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist


class WaypointUpdater(object):
    """A waypoint updater node implemented as a Python class."""

    def __init__(self):
        """
        Constructor.

        - Subscribes to current_pose and base_waypoints
        - Gets the lookahead parameter ~lookahead_wps
        - Advertises to final_waypoints
        - Sets up class variables
        """
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.lookahead_wps = rospy.get_param('~lookahead_wps', 200)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.currpose = None
        self.curr_waypoints = None

    def pose_cb(self, msg):
        """Callback for the curr_pose just stores it for later use."""
        self.currpose = msg.pose.position

    def waypoints_cb(self, lanemsg):
        """
        Callback for base_waypoints finds the waypoint.

        Finds the waypoint closes to the current pose, then publishes the next lookahead_wps
        waypoints after that one to final_waypoints.
        """
        if self.currpose is None:
            return

        self.curr_waypoints = lanemsg.waypoints
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        mindist = 1000000
        start_idx = 0

        for i in range(len(self.curr_waypoints)):
            a = self.curr_waypoints[i].pose.pose.position
            b = self.currpose
            dist = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
            if dist < mindist and (a.x > b.x):
                start_idx = i
                mindist = dist
            if start_idx == (len(self.curr_waypoints) - 1):
                start_idx = 0

        idx = 0
        reset = 0
        # Collecting the waypoints ahead of the car.
        # Wrap around when we reach the end.
        for i in range(self.lookahead_wps):
            idx = (start_idx + i) % len(self.curr_waypoints)
            lane.waypoints.append(self.curr_waypoints[idx])

        self.final_waypoints_pub.publish(lane)

    def traffic_cb(self, msg):
        """Callback for the traffic_waypoint message."""
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        """Callback for the obstacle_waypoint message."""
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_updater')
        node = WaypointUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
