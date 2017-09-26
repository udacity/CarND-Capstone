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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

distance3d = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
distance2d = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb) # Implement later
        self.all_waypoints = []
        self.waypoints_ahead = []

        self.last_pose = None
        self.next_waypoint_ahead = None
        self.next_traffic_light = None

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.next_waypoint_ahead_pub = rospy.Publisher(
            'next_waypoint_ahead', Int32, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.last_pose = msg.pose
        if len(self.all_waypoints) > 0: # base waypoints published
            next_waypoint_ahead = self.get_next_waypoint_ahead()
            if next_waypoint_ahead != self.next_waypoint_ahead:
                waypoints_ahead = self.get_waypoints_ahead(next_waypoint_ahead)
                self.adapt_target_velocities(waypoints_ahead)
                self.publish(next_waypoint_ahead, waypoints_ahead)

    def waypoints_cb(self, waypoints):
        # reassign all waypoints of the track 
        # note: this should be a circular track
        self.all_waypoints = list(waypoints.waypoints)

    def traffic_cb(self, msg):
        # - msg is an index to all_waypoints[] indicating the position
        #   of the stop line of the next red light
        # - in case no red light is in sight, the value is -1
        self.next_traffic_light = msg.data
        if self.next_traffic_light != -1:
            rospy.loginfo("Traffic light at %s" % self.next_traffic_light)

    def obstacle_cb(self, msg):
        # TODO Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def adapt_target_velocities(self, waypoints_ahead):
        if self.next_traffic_light in (None, -1):
            return
        if self.next_waypoint_ahead > self.next_traffic_light:
            return
        rospy.loginfo("Setting velocities so we stop come to a halt")
        rospy.loginfo("Red light at %s" % (self.next_traffic_light))
        rospy.loginfo("Next waypoint ahead %s" % (self.next_waypoint_ahead))
        next_waypoint_velocity = self.get_waypoint_velocity(self.all_waypoints[self.next_waypoint_ahead])
        rospy.loginfo("Current velocity target for next waypoint: %s" % (next_waypoint_velocity))
        distance_to_stop = self.waypoint_distance(self.all_waypoints, self.next_waypoint_ahead, self.next_traffic_light)
        rospy.loginfo("Distance of next waypoint to red light: %s" % (distance_to_stop))

        if distance_to_stop > 80:
            return
        
        for i in range(self.next_traffic_light - self.next_waypoint_ahead):
            velocity_adjusted = next_waypoint_velocity - (i+1) * next_waypoint_velocity /(self.next_traffic_light - self.next_waypoint_ahead)
            rospy.loginfo("Setting velocity of waypoint index %s to %s" % (i, velocity_adjusted))
            self.set_waypoint_velocity(waypoints_ahead, i, velocity_adjusted)

    def waypoint_distance(self, waypoints, i1, i2):
        dist = 0
        for i in range(i1, i2):
            dist += distance3d(
                waypoints[i].pose.pose.position,
                waypoints[i+1].pose.pose.position)
        return dist

    def pose_distance(self, pose1, pose2):
        return distance2d(pose1.position, pose2.position)

    def find_first_waypoint_ahead(self):
        return 275 # TODO search this

    def yaw_from_quaternion(self, q):
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return euler[2]
 
    def coordinate_transform(self, x, y, yaw):
        # TODO explain to myself why negative yaw works!
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        return x*cos_yaw-y*sin_yaw, x*sin_yaw+y*cos_yaw
        
    def car_has_passed_waypoint(self, car_pose, waypoint):
        # waypoint is the origo, and waypoint yaw x-axis direction
        x, y = self.coordinate_transform(
            car_pose.position.x - waypoint.pose.pose.position.x,
            car_pose.position.y - waypoint.pose.pose.position.y,
            self.yaw_from_quaternion(waypoint.pose.pose.orientation))
        return (x > 0.0) # car is ahead of the waypoint
       
    def add_waypoint_index(self, i, n):
        return (i + n) % len(self.all_waypoints)

    def find_next_waypoint_ahead(self):
        nwpa1 = self.add_waypoint_index(self.next_waypoint_ahead, 1)
        nwpa2 = self.add_waypoint_index(nwpa1, 1)
        car = self.last_pose
        dist1 = self.pose_distance(car, self.all_waypoints[nwpa1].pose.pose)
        dist2 = self.pose_distance(car, self.all_waypoints[nwpa2].pose.pose)

        # go forward in the waypoint list in order to find local minimum distance
        while dist2 < dist1:
            dist1 = dist2
            nwpa1 = nwpa2
            nwpa2 = self.add_waypoint_index(nwpa2, 1)
            dist2 = self.pose_distance(car, self.all_waypoints[nwpa2].pose.pose)

        # take next if we have already passed the closest one
        if self.car_has_passed_waypoint(car, self.all_waypoints[nwpa1]):
            nwpa1 = nwpa2

        return nwpa1

    def get_next_waypoint_ahead(self):
        cwpa = self.next_waypoint_ahead # current waypoint ahead
        next_waypoint_ahead = cwpa
        if cwpa == None:
            next_waypoint_ahead = self.find_first_waypoint_ahead()
        elif self.car_has_passed_waypoint(self.last_pose, self.all_waypoints[cwpa]):
            next_waypoint_ahead = self.find_next_waypoint_ahead()

        # debugging
        if next_waypoint_ahead != cwpa:
            dist = self.pose_distance(
                self.last_pose,
                self.all_waypoints[next_waypoint_ahead].pose.pose)
            dist2 = self.pose_distance(
                self.last_pose,
                self.all_waypoints[next_waypoint_ahead-1].pose.pose)
            dist3 = self.pose_distance(
                self.all_waypoints[next_waypoint_ahead-1].pose.pose,
                self.all_waypoints[next_waypoint_ahead].pose.pose)
            rospy.loginfo(
                "waypoint_updater: next waypoint %s, car position (%s,%s,%s)",
                next_waypoint_ahead,
                self.last_pose.position.x,
                self.last_pose.position.y,
                self.yaw_from_quaternion(self.last_pose.orientation))

        return next_waypoint_ahead

    def get_waypoints_ahead(self, i):
        waypoints = []

        if self.next_waypoint_ahead == i:
            waypoints = self.waypoints_ahead
        else: 
            self.next_waypoint_ahead = i

            first_waypoint = self.next_waypoint_ahead
            last_waypoint  = self.add_waypoint_index(first_waypoint, LOOKAHEAD_WPS)

            if first_waypoint < last_waypoint:
                waypoints = self.all_waypoints[first_waypoint:last_waypoint+1]
            else:
                waypoints = self.all_waypoints[first_waypoint:]
                waypoints.extend(self.all_waypoints[:last_waypoint+1])

            self.waypoints_ahead = waypoints
        
        return waypoints

    def publish(self, next_waypoint_ahead, waypoints_ahead):
        lane = Lane()

        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)

        lane.waypoints = waypoints_ahead

        self.next_waypoint_ahead_pub.publish(next_waypoint_ahead)

        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
