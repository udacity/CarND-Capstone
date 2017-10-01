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

CONVERSION_FACTOR = 0.447039 # Factor for converting MPH (miles per hour) in MPS (meters per second)
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. Number to adapt
MAX_SPEED = 10 * CONVERSION_FACTOR #Speed limit (in MPS)

class WaypointUpdater(object):
    def __init__(self):
        # initialize the node waypoint_updater
        rospy.init_node('waypoint_updater')

        # subscribe to the topic /current_pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # subscribe to the topic /base_waypoints
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # subscribe to the topic /traffic_waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # subscribe to the topic /obstacle_waypoint
        #rospy.Subscriber('/obstacle_waypoint', , self.obstacle_cb)

        #publisher to the topic final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # initialize self.pose (current vehicle position and orientation)
        self.current_pose = None
        # initialize self.waypoints
        self.waypoints = None
        # initialize self.frame_id
        self.frame_id = ''
        # initialize self.lights
        #self.lights = []
        self.waypoint_on_red_light = -1

        # we use the method rospy.spin() to block until a shutdown request is received from the node
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #set pose to msg.pose
        self.current_pose = msg.pose
        self.frame_id = msg.header.frame_id
        self.publish()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # set waypoints
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        if msg.data > -1:
            self.waypoint_on_red_light = msg.data
            self.publish()


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

    # funtion to compute the closest waypoint to our current position
    def closest_waypoint(self,pose,waypoints):
        closest_len = 100000
        closest_waypoint = 0
        # define dl lambda function (distance between two points in 2D)
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        # make a loop to determine the closest waypoint to our position
        for i,waypoint in enumerate(self.waypoints):
            dist = dl(pose.position,waypoint.pose.pose.position)
            if dist < closest_len:
                closest_len = dist
                closest_waypoint = i

        return closest_waypoint

    # funtion to compute the next waypoint to our current position
    # the next waypoint is the closest point in front of our car
    def next_waypoint(self,pose,waypoints):
        # compute the closest waypoint to our position
        closest_waypoint = self.closest_waypoint(pose,waypoints)
        # determine map_x and map_y
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y
        # compute the heading 
        heading = math.atan2(map_y-pose.position.y,map_y-pose.position.x)
        # compute yaw using transformations euler from quaternion
        orientations = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        _,_,yaw = tf.transformations.euler_from_quaternion(orientations)
        # compute the angle difference between yaw and heading
        angle = abs(yaw - heading)
        if angle > (math.pi/4):
            closest_waypoint += 1

        return closest_waypoint

    def publish(self):
        if self.current_pose is not None:
            # compute the index of the next waypoint
            index = self.next_waypoint(self.current_pose,self.waypoints)
            # obtain the list of waypoints to publish as a subset of waypoints
            # (from next waypoint index to this index plus LOOKAHEAD_WPS)
            pub_waypoints = self.waypoints[index:index+LOOKAHEAD_WPS]
            # set the max speed of the list of waypoints
            for i in range(len(pub_waypoints) - 1):
                # set the MAX_SPEED as speed for each waypoint in the list
                self.set_waypoint_velocity(pub_waypoints,i,MAX_SPEED)
            # initialize lane object
            lane = Lane()
            # set lane frame_id
            lane.header.frame_id = self.frame_id
            # set lane time
            lane.header.stamp = rospy.Time(0)
            # set lane waypoints
            lane.waypoints = pub_waypoints

            # publish lane
            self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
