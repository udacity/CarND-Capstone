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

LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obtacle_waypoint', Waypoint, self.obstacle_cb) # Implement later

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.all_waypoints = []
        self.last_pose = None
        self.next_waypoint_ahead = None
        self.next_traffic_light = None

        rospy.spin()

    def pose_cb(self, msg):
        self.last_pose = msg.pose
        if len(self.all_waypoints) > 0:
            next_waypoint_ahead = self.get_next_waypoint_ahead()
            if (next_waypoint_ahead != self.next_waypoint_ahead):
                dist = self.pose_distance(msg.pose, self.all_waypoints[next_waypoint_ahead].pose.pose)
                rospy.loginfo("waypoint_updater: next waypoint %s, distance %s", next_waypoint_ahead, dist)
                self.next_waypoint_ahead = next_waypoint_ahead
                self.publish_way_ahead()

    def waypoints_cb(self, waypoints):
        # reassign all waypoints of the route
        # note: this should be a circular track
        self.all_waypoints = list(waypoints.waypoints)

    def traffic_cb(self, msg):
        # - msg is an index to all_waypoints array indicating the position
        #   of stop line of the next red light
        # - in case no red light is in sight, the value is -1
        self.next_traffic_light = msg

    def obstacle_cb(self, msg):
        # TODO Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def pose_distance(self, pose1, pose2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(pose1.position, pose2.position)

    def waypoint_distance(self, waypoints, i1, i2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(i1, i2):
            dist += dl(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist

    def get_first_waypoint_ahead(self):
        return 274 # TODO search this

    def yaw_from_quaternion(self, q):
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return euler[2]
 
    def car_has_passed_waypoint(self, car_pose, waypoint):
        x0 = car_pose.position.x
        y0 = car_pose.position.y
        x1 = waypoint.pose.pose.position.x
        y1 = waypoint.pose.pose.position.y

        # waypoint direction in global coordinates
        dirg = math.atan2(x1-x0, y1-y0)
        # waypoint direction in car coordinates
        dirc = self.yaw_from_quaternion(car_pose.orientation) - dirg
        # if abs waypoint direction is > pi/2, car has passed the waypoint
        return abs(dirc) > math.pi/2

    def add_waypoint_index(self, i, n):
        return (i + n) % len(self.all_waypoints)

    def get_next_waypoint_ahead(self):
        cwpa = self.next_waypoint_ahead # current next waypoint ahead
        next_waypoint_ahead = cwpa
        if cwpa == None:
            next_waypoint_ahead = self.get_first_waypoint_ahead()
        elif self.car_has_passed_waypoint(self.last_pose, self.all_waypoints[cwpa]):
            # this makes sense only if we assume that vehicle follows the track
            # otherwise need to look for the closest waypoint
            next_waypoint_ahead = self.add_waypoint_index(cwpa, 1)
            # TODO check that the new waypoint makes sense
        return next_waypoint_ahead

    def publish_way_ahead(self):
        lane = Lane()

        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)

        first_waypoint = self.next_waypoint_ahead
        last_waypoint  = self.add_waypoint_index(first_waypoint, LOOKAHEAD_WPS)

        if first_waypoint < last_waypoint:
            lane.waypoints = self.all_waypoints[first_waypoint:last_waypoint+1]
        else:
            lane.waypoints = self.all_waypoints[first_waypoint:]
            lane.waypoints.extend(self.all_waypoints[:last_waypoint+1])

        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
