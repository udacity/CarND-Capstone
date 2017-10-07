#!/usr/bin/env python

import tf
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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = []

        rospy.spin()

    def pose_cb(self, msg):
        # If waypoints not yet received, do nothing
        if len(self.base_waypoints) == 0:
            print("didn't get base waypoints yet")
            return

        # Find car position and direction
        car_x = msg.pose.position.x
        car_y = msg.pose.position.y
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        car_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        #print("Car is at %f, %f and facing %f" % (car_x, car_y, car_yaw))
        
        # Find nearest waypoint
        min_distance = float("inf")
        min_distance_waypoint = 0
        for i in range(0, len(self.base_waypoints)):
            waypoint = self.base_waypoints[i]
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            distance = math.sqrt((car_x-waypoint_x)**2 + (car_y-waypoint_y)**2)
            if distance < min_distance:
                min_distance = distance
                min_distance_waypoint = i

        # Nearest waypoint may be behind us!  Check this by transforming waypoint to
        # car's coordinate system.
        waypoint_x = self.base_waypoints[min_distance_waypoint].pose.pose.position.x
        waypoint_y = self.base_waypoints[min_distance_waypoint].pose.pose.position.y
        #print("Closest waypoint is %d, at %f, %f (distance %f)" % (min_distance_waypoint, waypoint_x, waypoint_y, min_distance))
        transformed = self.transform(waypoint_x, waypoint_y, -car_x, -car_y, -car_yaw)
        #print("Transformed: %f, %f" % (transformed[0], transformed[1]))
        
        # If x coordinate of nearest waypoint is negative, that means it's behind us.
        # Use next waypoint instead.
        if transformed[0] < 0:
            #print("Closest waypoint is behind us, using the next one")
            min_distance_waypoint = min_distance_waypoint+1

        # Publish LOOKAHEAD_WPS waypoints
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.base_waypoints[min_distance_waypoint:min_distance_waypoint+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        print("got %d waypoints" % len(waypoints.waypoints))
        self.base_waypoints = waypoints.waypoints
        pass

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

    # transform coordinate systems
    def transform(self, x, y, x_offset, y_offset, rotation):
        x = x + x_offset
        y = y + y_offset
        new_x = x * math.cos(rotation) - y * math.sin(rotation)
        new_y = x * math.sin(rotation) + y * math.cos(rotation)
        return [new_x, new_y]


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
