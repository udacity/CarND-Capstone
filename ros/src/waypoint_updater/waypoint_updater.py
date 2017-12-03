#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion

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

        rospy.spin()

    def pose_cb(self, msg):

        # get nextWaypoint in front of car
        nextWaypoint = self.getNextWaypoint(msg.pose)

        self.final_waypoints = self.base_waypoints.waypoints[nextWaypoint:(nextWaypoint + LOOKAHEAD_WPS)]

        self.publish()


    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints


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

    def euclidianDistance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))


    def publish(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)


    def getClosestWaypoint(self, pose):

        closestLen = 100000
        closestWaypoint = 0

        # get car x and y from pose message
        car_x = pose.position.x
        car_y = pose.position.y

        for i in range(len(self.base_waypoints.waypoints)):

            # get x and y for waypoint being processed
            waypoint_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            waypoint_y = self.base_waypoints.waypoints[i].pose.pose.position.y

            distance = self.euclidianDistance(car_x, car_y, waypoint_x, waypoint_y)

            if distance < closestLen:
                closestLen = distance
                closestWaypoint = i

        return closestWaypoint


    def getNextWaypoint(self, pose):

        # car location and heading
        car_angles = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        car_heading = car_angles[2]
        car_x = pose.position.x
        car_y = pose.position.y

        closestWaypoint = self.getClosestWaypoint(pose)

        waypoint_x = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.x
        waypoint_y = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.y

        # this calculates heading between carpoint/position and waypoint in radians
        heading = math.atan2((waypoint_y - car_y),(waypoint_x - car_x))

        # angle between car heading and heading
        angle = abs(car_heading - heading)

        if angle > math.pi:
            closestWaypoint += 1

        return closestWaypoint



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
