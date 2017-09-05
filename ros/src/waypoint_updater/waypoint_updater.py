#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # The rate of the `/final_waypoints` publishing will be the same as the 
        # `/current_pose` than `/base_waypoints`, since the second one rarely change.
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # keep the base_waypoints since they are repeated every cycle
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        cur_pose =  msg.pose
        # need to benchmark whether going through all base points
        # is fast enough, otherwise, we may need to keep the points
        # that have been passed
        if self.base_waypoints is None: return

        next_wp_index = None
        for i, waypoint in enumerate(self.base_waypoints):
            if self.waypoint_is_aheadof_car(waypoint, cur_pose):
                # next way point as the first one ahead of car
                next_wp_index = i
                break
        # rospy.loginfo("found next_wp_index = %i" % next_wp_index)
        if next_wp_index is not None:
            lane = Lane()
            # lane.header.frame_id = '/world'
            # lane.header.stamp = rospy.Time(0)
            lane.waypoints = self.base_waypoints[next_wp_index:next_wp_index+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # 10902 waypoints for the simulation env
        if self.base_waypoints is None:
            self.base_waypoints = waypoints.waypoints

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
    def waypoint_is_aheadof_car(self, waypoint, car_pose):
        """Return if a waypoint is ahead of the car.
        Ideally it should use the Frenet coordinates? But here we just use x
        of world coordinates
        """
        wp_x = waypoint.pose.pose.position.x
        car_x = car_pose.position.x
        # rospy.loginfo("wp_x=%g, car_x=%g" % (wp_x, car_x))
        return wp_x >= car_x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
