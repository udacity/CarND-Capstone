#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.base_lane = None
        self.traffic_wp = None
        self.obstacle_wp = None

        rospy.spin()

    def pose_cb(self, msg):
        #rospy.loginfo("pose_cb timestamp %s x=%d y=%d z=%d", msg.header.stamp, msg.pose.position.x,
        #              msg.pose.position.y, msg.pose.position.z)
        if self.base_lane != None:
            quaternion = (msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            #rospy.loginfo("roll %f pitch %f yaw %f", roll, pitch, yaw)
            
            # the course hits an inflection point at x=2339 yaw=90+ at which point x starts to decrease
            # next inflection at x=155 yaw=270+ at which point x starts to increase
            # in front of the car = increasing x for yaw between 270 and 90 and decreasing x from 90 to 270
            pub_waypoints = []
            wp_cnt = 0
            fwd = True
            if yaw > 90.0 and yaw <= 270.0:  # exactly 90/270 handling is a bit arbitrary
                fwd = False
            for wp in self.base_lane.waypoints:
                if ((fwd and wp.pose.pose.position.x > msg.pose.position.x) or
                (not fwd and wp.pose.pose.position.x < msg.pose.position.x)) and wp_cnt < LOOKAHEAD_WPS:
                    wp_cnt = wp_cnt + 1
                    #rospy.loginfo("wp %d: x=%f y=%f", wp_cnt, wp.pose.pose.position.x, wp.pose.pose.position.y)
                    pub_waypoints.append(wp)

            # Add velocity command to waypoints
            # TESTING
            for wp_cnt in range(10):
                if len(pub_waypoints) > wp_cnt:
                    self.set_waypoint_velocity(pub_waypoints, wp_cnt, 1)
            
            l = Lane()
            l.header = msg.header
            l.waypoints = pub_waypoints
            self.final_waypoints_pub.publish(l)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("waypoints_cb got %d waypoints", len(waypoints.waypoints))
        self.base_lane = waypoints
        pass

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.traffic_wp = msg.data
        pass

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_wp = msg.data
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
