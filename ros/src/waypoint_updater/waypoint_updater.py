#!/usr/bin/env python

import math
import rospy
import tf
import waypoint_helper as Helper
from std_msgs.msg import Int32
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
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
DISTANCE_STOP_AT_TRAFFIC = 28 # stop distance before traffic light
MAX_DECEL = 1.0 # max deceleration in ms-2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=2)

        self.base_waypoints = None
        self.current_pose = None
        self.num_waypoints = 0
        self.closest_waypoint = None
        self.traffic = -1

        self.publish()
        rospy.spin()

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            final_waypoints = self.get_final_waypoints()
            if final_waypoints:
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        """ Callback for current vehicle pose """
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        """ Callback for base waypoints """
        self.base_waypoints = waypoints.waypoints
        self.num_waypoints = len(self.base_waypoints)
        self.waypoints_sub.unregister()

    def traffic_cb(self, msg):
        """ Callback for traffic lights """
        self.traffic = int(msg.data)

    def obstacle_cb(self, msg):
        """ Callback for obstacles """
        pass

    def get_final_waypoints(self):
        if not self.current_pose or not self.base_waypoints:
            return None

        final_waypoints, self.closest_waypoint = Helper.look_ahead_waypoints(self.current_pose,
                                                                             self.base_waypoints,
                                                                             self.closest_waypoint,
                                                                             LOOKAHEAD_WPS)

        if self.traffic != -1:
            final_waypoints = Helper.decelerate_waypoints(self.base_waypoints,
                                                          final_waypoints,
                                                          self.closest_waypoint,
                                                          self.traffic,
                                                          DISTANCE_STOP_AT_TRAFFIC)

        rospy.logout("closest %d, traffic %d", self.closest_waypoint, self.traffic)
        info = "speed for waypoint: " + ", ".join("%05.2f" % wp.twist.twist.linear.x for wp in final_waypoints[:10])
        rospy.logout(info)

        return final_waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
