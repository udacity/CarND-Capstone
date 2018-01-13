#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import os
import tf
from numpy import random

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
PUBLISH_RATE = 20 # Publishing rate (Hz)
SAFE_DECEL_FACTOR = 0.1 # Multiplier to the decel limit.

max_local_distance = 20.0 # Max waypoint distance we admit for a local minimum (m)

class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.logdebug('Object Waypoints updater created')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #rospy.logdebug('Current position x:', current_pose.Pose.position.x)
        
        base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.logdebug('Base waypoints ', base_waypoints)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.logdebug('Traffic waypoints ', traffic_waypoint)

        obstacle_waypoint = rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)
        #rospy.logdebug('Obstacle waypoints ', obstacle_waypoint)
        
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below	
        self.cruise_speed = None
        self.decel_limit = None
        self.accel_limit = None
        self.car_position = None
        self.waypoints = []
        self.final_waypoints = []
        self.tl_state = None
        self.car_curr_vel = None
        self.nextWaypoint = None
        self.car_yaw = None

        self.work()
        #rospy.spin()

    # Position of the car
    def pose_cb(self, msg):
        car_pose = msg.pose
        self.car_position = car_pose.position 
        car_orientation = car_pose.orientation
        quaternion = (car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]

    # Waypoints (not complete yet)
    def waypoints_cb(self, msg):
        return msg.waypoints

    # Callback for /traffic_waypoint message. (not complete yet)
    def traffic_cb(self, msg):
        return msg.data

    # Callback for /obstacle_waypoint message. We will implement it later
    def obstacle_cb(self, msg):
        pass

    # Callback for /current_velocity
    def current_velocity_cb(self, msg):
        curr_lin = [msg.twist.linear.x, msg.twist.linear.y]
        self.car_curr_vel = math.sqrt(curr_lin[0]**2 + curr_lin[1]**2)

    # function work
    def work(self):
        
        rate = rospy.Rate(PUBLISH_RATE)
        # ROS parameters
        self.cruise_speed = self.kmphToMph(rospy.get_param('~/waypoint_loader/velocity', 40.0))
        self.decel_limit = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1) 
        
        env_velocity = os.getenv('VELOCITY','0')
        if float(env_velocity) > 0.01:
           self.cruise_speed = self.kmphToMph(float(env_velocity))

        while not rospy.is_shutdown():

                if (self.car_position != None and self.waypoints != None and self.car_curr_vel != None):  ##we need to add here the validation of the traffic light

                    self.safe_distance = (self.car_curr_vel ** 2)/(2 * self.decel_limit * SAFE_DECEL_FACTOR)
                    self.nextWaypoint = self.NextWaypoint(self.car_position, self.car_yaw, self.waypoints)
                    self.car_action = self.DesiredAction(self.tl_idx, self.tl_state, self.closestWaypoint, self.waypoints)

                    self.generateFinalWaypoints(self.nextWaypoint, self.waypoints, self.car_action)
                    self.publish()
                else:
                       rand = random.uniform(0,1) ## avoiding writting all the time in the logs
                       if self.car_position == None and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /current_pose not received")
                       if self.waypoints == None and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /base_waypoints not received")
                       if self.car_curr_vel == None  and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /current_velocity not received")
                rate.sleep()

    # generation of Final Waypoints
    def generateFinalWaypoints(self, nextWaypoint, waypoints, action):
        
        self.final_waypoints = []
        
        if (action == "GO"):
           self.GoWaypoints(nextWaypoint, waypoints)

    def GoWaypoints(self, nextWaypoint, waypoints):
        
        init_vel = self.car_curr_vel
        end = nextWaypoint + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        a = ACC_FACTOR * self.accel_limit
        for i in range(nextWaypoint, end):
            dist = self.distance(waypoints, nextWaypoint, i+1)
            velocity = math.sqrt(init_vel**2 + 2 * a * dist)
            if velocity > self.cruise_speed:
               velocity = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[i])



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
    
    # Distance to any point
    def distanceAny(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

    # function Closest Waypoints
    def ClosestWaypoint(self, position, waypoints):
        closestLen = float("inf")
        closestWaypoint = 0
        dist = 0.0
        for i in range(0, len(waypoints)):
                x = position.x
                y = position.y
                map_x = waypoints[i].pose.pose.position.x
                map_y = waypoints[i].pose.pose.position.y
                dist = self.distance_any(x, y, map_x, map_y)
                if (dist < closestLen):
                        closestLen = dist
                        closestWaypoint = i
        return closestWaypoint

    # function Next Waypoints
    def NextWaypoint(self, position, yaw, waypoints):

	closestWaypoint = self.ClosestWaypoint(position, waypoints)

        map_x = waypoints[closestWaypoint].pose.pose.position.x
        map_y = waypoints[closestWaypoint].pose.pose.position.y

        heading = math.atan2((map_y - position.y), (map_x - position.x))

        angle = abs(yaw - heading)

	if (angle > math.pi/4):
		closestWaypoint += 1 
                if (closestWaypoint > len(waypoints)-1):
                    closestWaypoint -= 1

	return closestWaypoint

    #function to convert from kilometers to milles
    def kmphToMph(self, kmph):
        return 0.278 * kmph

    #Action to execute
    def DesiredAction(self, closestWaypoint, waypoints): 
        action = "GO"
        return action 



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
