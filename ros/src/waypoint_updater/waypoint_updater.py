#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        self.publish()

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish(self):
        if self.base_waypoints is not None:
            closest_waypoint_idx = get_closest_waypoint()
            m = min(len(base_waypoints), closest_waypoint_idx + LOOKAHEAD_WPS)
            final_waypoints = base_waypoints[closest_waypoint_idx:m]
            published_lane = Lane()
            published_lane.header.stamp = rospy.Time.now()
            published_lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(published_lane)

    def get_closest_waypoint(self):
        pose = self.pose
        current_position = pose.position
        closest_gap = float('inf')
        closest_gap_idx = 0

        for idx, waypoint in enumerate(self.base_waypoints):
            waypoint_position = waypoint.pose.pose.position
            dx = current_position.x - waypoint_position.x
            dy = current_position.y - waypoint_position.y
            gap = dx*dx + dy*dy

            if gap < closest_gap:
                roll, pitch, yaw = tf.transformations.euler_from_quarternion(
                    [pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w])
                heading_x = pose.position.x
                heading_y = pose.position.y
                gap_x = waypoint.pose.pose.position.x - heading_x
                gap_y = waypoint.pose.pose.position.y - heading_y
                x = gap_x * cos(0 - yaw) - gap_y * sin(0 - yaw)
                if x > 0:
                    closest_gap = gap
                    closest_gap_idx = idx
        return closest_gap_idx

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """ Get total distance between two waypoints given their index"""
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
