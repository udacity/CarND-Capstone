#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import tf
from multiprocessing import Lock

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
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_msg = None
        self.prev_closest_idx = 0
        #
        # lock is required because in rospy subscriber's callbacks are executed in separate threads
        # https://answers.ros.org/question/110336/python-spin-once-equivalent/
        self.lock = Lock()

        rospy.spin()

    def pose_cb(self, msg):
        with self.lock:
            waypoints_msg = self.waypoints_msg

        waypoints = waypoints_msg.waypoints
        if waypoints is not None:
            #
            # find a closest waypoint
            #
            closest_idx = self.prev_closest_idx
            if closest_idx > 5:
                closest_idx -= 5
            closest_distance = 1e6
            distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)  # + (a.z - b.z) ** 2)
            for idx in range(closest_idx, len(waypoints)):
                point = waypoints[idx]
                dist = distance(point.pose.pose.position, msg.pose.position)
                if dist < closest_distance:
                    closest_distance = dist
                    closest_idx = idx
                else:
                    break
            self.prev_closest_idx = closest_idx
            point = waypoints[closest_idx]

            #
            # adjust it by heading
            #
            q = point.pose.pose.orientation
            position = point.pose.pose.position
            _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            heading = math.atan2((position.y - msg.pose.position.y), (position.x - msg.pose.position.x))

            if abs(yaw - heading) > (math.pi / 4):
                closest_idx += 1

            #
            # compose final waypoints
            #
            final_waypoints = Lane()
            final_waypoints.header.frame_id = self.waypoints_msg.header.frame_id
            final_waypoints.header.stamp = rospy.Time.now()
            final_waypoints.waypoints = waypoints[closest_idx:min(len(waypoints), closest_idx+LOOKAHEAD_WPS)]

            #
            # publish it
            #
            self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, msg):
        with self.lock:
            if self.waypoints_msg is None or self.waypoints_msg.header.stamp != msg.header.stamp:
                self.waypoints_msg = msg
                self.prev_closest_idx = 0

        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo('traffic_cb called')
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        rospy.loginfo('obstacle_cb called')
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
