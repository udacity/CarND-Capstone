#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish
MAX_DECEL = 0.5
STOP_DISTANCE = 5.0
TARGET_SPEED_MPH = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below        
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.cur_pose = None
        self.waypoints = None  
        self.red_light_waypoint = None      
        self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        self.cur_pose = msg.pose 
        if self.waypoints is not None:                
            self.publish()

    def waypoints_cb(self, lane):
        # >> Do this once and not all the time
        if self.waypoints is None:
            self.waypoints = lane.waypoints            

    def traffic_cb(self, msg):
        self.red_light_waypoint = msg.data        
        rospy.loginfo("Detected light: " + str(msg.data))
        if self.red_light_waypoint > -1:
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

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
    
    def closest_waypoint(self, pose, waypoints):
        # >> Simply take the code from the path planning module and re-implement it here
        closest_len = 100000
        closest_waypoint = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

        for index, waypoint in enumerate(self.waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint = index
                
        return closest_waypoint

    def next_waypoint(self, pose, waypoints):
        # >> Same concepts from path planning in here
        closest_waypoint = self.closest_waypoint(pose, waypoints)
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y

        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)

        if angle > (math.pi / 4):        
            closest_waypoint += 1
        
        return closest_waypoint

    def decelerate(self, waypoints, redlight_index):
        if len(waypoints) < 1:
            return []
          
        first = waypoints[0]
        last = waypoints[redlight_index]

        last.twist.twist.linear.x = 0.
        total_dist = self.distance(first.pose.pose.position, last.pose.pose.position)
        start_vel = first.twist.twist.linear.x

        # >> Start from the waypoint before last and go backwards
        for index, wp in enumerate(waypoints):

            if index > redlight_index:
                vel = 0
            else:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0, dist - STOP_DISTANCE)
                vel  = math.sqrt(2 * MAX_DECEL * dist) 
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def publish(self):        
        if self.cur_pose is not None:
            next_waypoint_index = self.next_waypoint(self.cur_pose, self.waypoints)
            lookahead_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]

            if self.red_light_waypoint is None or self.red_light_waypoint < 0:
                # >> Set the velocity for lookahead waypoints
                for i in range(len(lookahead_waypoints) - 1):                
                    # >> Convert 10 miles per hour to meters per sec
                    self.set_waypoint_velocity(lookahead_waypoints, i, (TARGET_SPEED_MPH * 1609.34) / (60 * 60))
            else:                
                redlight_lookahead_index = max(0, self.red_light_waypoint - next_waypoint_index)
                lookahead_waypoints = self.decelerate(lookahead_waypoints, redlight_lookahead_index)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = lookahead_waypoints
            
            self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')