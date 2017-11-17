#!/usr/bin/env python
import sys                      # for redirect stderr
import rospy

import copy                     # for deepcopy
import numpy as np              # for polyfit and poly1d

import math

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray
from waypoint_lib.waypoint_tracker import WaypointTracker

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

LOOKAHEAD_WPS = 30 # 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_TIME_THRESHOLD = 4 # seconds, change from 5 to 4
SAEF_TURNING_SPEED = 3.0       # meters/second

DANGER_TURNING_ANGLE = math.pi/4  # 30 degree
MPH_to_MPS = 1609.344/3600.0 # 1 mile = 1609.344 1 hour = 3600 seconds

def publish_Lane(publisher, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        publisher.publish(lane)
def distance_two_indices(waypoints, i, j):
  a = waypoints[i].pose.pose.position
  b = waypoints[j].pose.pose.position
  return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(WaypointTracker):
    def __init__(self):
        # f = open("~/.ros/log/stderr.log", "w+") # not working here
        # self.original_stderr = sys.stderr
        # sys.stderr = f
        # self.stopped = False
        rospy.init_node('waypoint_updater')
        self.max_vel_mps = rospy.get_param('waypoint_loader/velocity')*MPH_to_MPS
        rospy.loginfo('max_vel_mps: %f' % self.max_vel_mps)
        self.loop_freq = rospy.get_param('~loop_freq', 2)
        # the frequency to process vehicle messages

        WaypointTracker.__init__(self)

        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint = None

        self.obstacle_waypoint = None
        self.current_velocity = None

        # self.traffic_lights = None
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # self.base_waypoints = None  # indicating the base_waypoints is not yet available
        # self.pose = None            # indicating that there is no message to process

        self.loop()
        #rospy.spin()

    def constant_policy_f(self, velocity, bound):
        xs = [-bound,   0.,       bound]
        ys = [velocity, velocity, velocity]
        return np.poly1d(np.polyfit(np.array(xs), np.array(ys), 2))
    def decleration_policy_f(self, ref_vel, bound):
        xs = []
        ys = []
    
        xs.append(-bound)
        ys.append(-0.1)
    
        xs.append(0.)
        ys.append(-0.2)
    
        # 5 meters away
        xs.append(5)
        ys.append(MPH_to_MPS*.5)
    
        # 10 meters away
        xs.append(10)
        ys.append(MPH_to_MPS*5)
    
        # 16 meters away
        xs.append(16)
        ys.append(MPH_to_MPS*5)
    
        # 2 seconds away or 24 meters away, whichever longer
        xs.append(max([ref_vel*2, 24]))
        ys.append(max([ref_vel*.2, MPH_to_MPS*5]))
    
        # 4 seconds away or 45 meters away, whichever longer
        xs.append(max([ref_vel*4, 45]))
        ys.append(max([ref_vel*.3, MPH_to_MPS*6]))
    
        # 6 seconds away or 65 meters away, whichever longer
        xs.append(max([ref_vel*6, 65]))
        ys.append(max([ref_vel*.5, MPH_to_MPS*10]))
    
        # 8 seconds away, normal speed
        xs.append(max([ref_vel*8, 85]))
        ys.append(ref_vel)
    
        # at the beginning, normal speed
        xs.append(bound)
        ys.append(ref_vel)
    
        return np.poly1d(np.polyfit(np.array(xs), np.array(ys), 3))
    def loop(self):
        rate = rospy.Rate(self.loop_freq)
        while not rospy.is_shutdown():
            if self.base_waypoints and self.pose:
                self.last_closest_front_waypoint_index = self.get_closest_waypoint(self.pose.pose)
                
                # generate final_waypoints
                waypoints_count = 0
                lookahead_dist = 0  # the accumulated distance of the looking ahead
                lookahead_time = 0  # the lookahead time
                
                final_waypoints = []
                accumulated_turning = 0
                dist_to_here_from_current = []
                
                # modulize the code to be less dependent
                j = self.last_closest_front_waypoint_index
                while (# (lookahead_time < LOOKAHEAD_TIME_THRESHOLD) and
                        (waypoints_count < LOOKAHEAD_WPS) and
                        (j < self.base_waypoints_num)):
                    waypoint = copy.deepcopy(self.base_waypoints[j])
                    dist_to_here_from_current.append(
                        self.dist_to_here_from_start[j]-
                        self.dist_to_here_from_start[self.last_closest_front_waypoint_index])
                
                    j = (j + 1) # % self.base_waypoints_num
                    waypoints_count += 1
                    final_waypoints.append(waypoint)
                # end of while (LOOKAHEAD_TIME_THRESHOLD <= lookahead_time) or (LOOKAHEAD_WPS <= waypoints_count)
                
                rospy.loginfo('Lookahead threshold reached: waypoints_count: %d; lookahead_time: %d; self.last_closest_front_waypoint_index: %d'
                              % (waypoints_count, lookahead_time, self.last_closest_front_waypoint_index))
                
                # publish to /final_waypoints, need to package final_waypoints into Lane message
                publish_Lane(self.final_waypoints_pub, final_waypoints)
                self.pose = None        # indicating this message has been processed
            # end of if self.base_waypoints and self.pose
            rate.sleep()
        # end of while not rospy.is_shutdow()
    def base_waypoints_cb(self, msg):
        WaypointTracker.base_waypoints_process(self, msg)
    
        global LOOKAHEAD_WPS        # might update it
        LOOKAHEAD_WPS = min(LOOKAHEAD_WPS, self.base_waypoints_num)
        # construct the velocity policy
        self.cruise_policy = self.constant_policy_f(self.max_vel_mps, LOOKAHEAD_WPS)
        self.stop_policy = self.constant_policy_f(-0.01, LOOKAHEAD_WPS)
        self.deceleration_policy = self.decleration_policy_f(self.max_vel_mps,
                                                             LOOKAHEAD_WPS)
    
        # set the deceleration when approaching the end of the track
        total_length = self.dist_to_here_from_start[self.base_waypoints_num-1]
        # the total distance from the start to finish
        for i in range(LOOKAHEAD_WPS):
            last_ith = self.base_waypoints_num - 1 - LOOKAHEAD_WPS+i
            dist_to_the_end = (total_length - self.dist_to_here_from_start[last_ith])
            expected_velocity = self.deceleration_policy(dist_to_the_end)
            self.base_waypoints[last_ith].twist.twist.linear.x = expected_velocity
        # end of for i in range(LOOKAHEAD_WPS)
    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data
    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
            return waypoint.twist.twist.linear.x
    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
            waypoints[waypoint].twist.twist.linear.x = velocity
    

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
