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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=2)

        self.waypoints = None
        self.current_pose = None
        self.num_waypoints = 0
        self.closest_waypoint = 0

        self.publish()
        rospy.spin()

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            next_waypoints = self.get_next_waypoints()
            if next_waypoints:
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = next_waypoints
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        """ Callback for current vehicle pose """
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        """ Callback for base waypoints """
        self.waypoints = waypoints.waypoints
        self.num_waypoints = len(self.waypoints)
        self.waypoints_sub.unregister()

    def traffic_cb(self, msg):
        """ Callback for traffic lights """
        pass

    def obstacle_cb(self, msg):
        """ Callback for obstacles """
        pass

    def get_next_waypoints(self):
        if not self.current_pose or not self.waypoints:
            return None

        waypoints_ahead = Helper.look_ahead_waypoints(self.current_pose, self.waypoints)

        return waypoints_ahead

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
