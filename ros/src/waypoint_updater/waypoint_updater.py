#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy
import tf.transformations   # to get Euler coordinates

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
        self.ego_pos = None
        self.wps = None
        self.final_wps = None
        self.first_pass = True
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        rospy.spin()

    def pose_cb(self, msg):
        self.ego_pos = msg.pose
        
        if self.wps is not None:	#Don't proceed until we have received waypoints
            
            #return the index of the closest waypoint ahead of us
            closest_idx_waypoint = self.closest_waypoint_ahead()

            #final waypoints is a subset of original set of waypoints
            self.final_wps.waypoints = self.wps.waypoints[closest_wp:closest_wp+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(self.final_wps)

    def waypoints_cb(self, waypoints):
        # Ensure we only get initial full list of waypoints as simulator keeps publishing
        # with patial list aftewards
        if self.wps is None:
            # We need to get a full copy as otherwise we just get a reference
            self.wps = copy.copy(waypoints)
            self.final_wps = copy.copy(waypoints)

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

    def closest_waypoint_ahead(self):
        ''' Return index of closest point ahead '''

        # Get car orientation
        car_x, car_y = self.ego_pos.position.x, self.ego_pos.position.y
        quaternion = (self.ego_pos.orientation.x, self.ego_pos.orientation.y,
                      self.ego_pos.orientation.z, self.ego_pos.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        car_yaw = euler[2]
        loginfo = 'Car yaw: {} | x: {} | y: {}'.format(car_yaw, car_x, car_y)

        # Define unit vector for car orientation in global (x, y) coordinates
        orient_x, orient_y = math.cos(car_yaw), math.sin(car_yaw)

        # Filter waypoints to keep only the ones ahead of us by checking scalar product
        waypoints_ahead = [(n, wp) for (n, wp) in enumerate(self.wps.waypoints)
                           if (orient_x * (wp.pose.pose.position.x - car_x) +
                           orient_y * (wp.pose.pose.position.y - car_y)) > 0]
        if not len(waypoints_ahead):
            rospy.logwarn("No points detected ahead of us")
        
        # Extract closest waypoint
        closest_waypoint = min(waypoints_ahead,
                               key = lambda wpidx: (wpidx[1].pose.pose.position.x - self.ego_pos.position.x) ** 2
                               + (wpidx[1].pose.pose.position.y - self.ego_pos.position.y) ** 2)
        closest_index = closest_waypoint[0]
        loginfo += '| Closest waypoint index: {}'.format(closest_index)
        rospy.loginfo_throttle(1, loginfo)

        return closest_index


if __name__ == '__main__':
    try:
	WaypointUpdater()	
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
