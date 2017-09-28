#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from itertools import chain
import tf
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
PUBLISH_PERIOD = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.last_pose_time = 0

        self.waypoints_received = False
        self.all_waypoints = []
        self.last_waypoint_idx = 0
        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):

        current_time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        if not self.waypoints_received:
            return
        if (current_time > self.last_pose_time + PUBLISH_PERIOD):
            self.last_pose_time = current_time

            concatenated = chain(range(self.last_waypoint_idx, len(self.all_waypoints)),
                                 range(self.last_waypoint_idx))

            x = msg.pose.position.x
            y = msg.pose.position.y
            quaternion = (
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # direction unit vector
            ux = math.cos(euler[2])
            uy = math.sin(euler[2])


            for idx in concatenated:
                pos = self.all_waypoints[idx].pose.pose
                wx = pos.position.x - x
                wy = pos.position.y - y
                intensity = math.sqrt(wx * wx + wy * wy)
                wx = wx / intensity
                wy = wy / intensity
                # if dot producti is > sqrt(2)/2 which corresponds to 45 degrees
                if (ux * wx + uy * wy) > math.sqrt(2) / 2:
                    next_points = Lane()
                    next_points.header.stamp = rospy.Time.now()
                    next_points.waypoints = [self.all_waypoints[i] for i in
                                             range(idx, (idx + LOOKAHEAD_WPS) % len(self.all_waypoints))]
                    self.final_waypoints_pub.publish(next_points)
                    self.last_waypoint_idx = idx
                    rospy.logwarn("First waypoint index " + str(self.last_waypoint_idx))
                    return


    def waypoints_cb(self, waypoints):
        rospy.loginfo("Waypoints received")
        self.all_waypoints = waypoints.waypoints
        self.waypoints_received = True



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
