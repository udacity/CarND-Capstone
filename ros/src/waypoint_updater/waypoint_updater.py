#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

TARGET_SPEED = 15

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.car_x = None
        self.car_y = None
        self.car_yaw = None

        rospy.spin()

    def pose_cb(self, msg):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        #need to know euler yaw angle for car orientation relative to waypoints
        #for quaternion transformation using https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        quaternion = [msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]

    def waypoints_cb(self, msg):
        # TODO: Implement
        if self.car_yaw is None:
            return

        closestWaypoint = self.get_closest_waypoint(msg.waypoints)
        lenWaypoints = len(msg.waypoints)
        final_waypoints_msg = Lane()
        for i in range(LOOKAHEAD_WPS):
            wp = msg.waypoints[(closestWaypoint + i) % lenWaypoints]
            new_final_wp = Waypoint()
            new_final_wp.pose = wp.pose
            #currently using constant speed to get car moving
            new_final_wp.twist.twist.linear.x = TARGET_SPEED
            final_waypoints_msg.waypoints.append(new_final_wp)
        self.final_waypoints_pub.publish(final_waypoints_msg)

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

    def get_closest_waypoint(self, waypoints):
        closestLen = 100000
        closestWaypoint = 0
        for i in range(len(waypoints)):
            wp = waypoints[i]
            dist = math.sqrt((self.car_x - wp.pose.pose.position.x)**2
                                + (self.car_y - wp.pose.pose.position.y)**2)
            if dist < closestLen:
                closestLen = dist
                closestWaypoint = i

        closest_wp = waypoints[closestWaypoint]
        heading = math.atan2(wp.pose.pose.position.y - self.car_y,
                                wp.pose.pose.position.x - self.car_x)
        angle = abs(self.car_yaw - heading)
        if (angle > math.pi/4):
            closestWaypoint += 1
        return closestWaypoint

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
