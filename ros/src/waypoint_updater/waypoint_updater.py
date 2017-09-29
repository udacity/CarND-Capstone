#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math
import numpy as np


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

MAX_DECEL     = 4.0
STOP_BUFFER   = 5.0

class WaypointUpdater(object):
    def __init__(self):

        # Properties
        self.prev_nrst_wp = 0 # total number of waypoints are 10902
        self.vehicle_pos = None
        self.current_linear_velocity = None
        self.upcoming_traffic_light_position = None
        self.waypoints = None
        self.braking = None
        self.decel = 1.0

        # Subscribers
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Point, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):

        # TODO: Implement
        if self.vehicle_pos != None and self.waypoints != None:
            waypoints = self.waypoints
            # try: # to catch when the error when self.vehicle_pos has not been created yet
            smallest_dist = float('inf')
            nearest_wp = 0

            # rospy.logwarn("previous nearest waypoint: %s", self.prev_nrst_wp)

            self.wp_num = len(waypoints.waypoints)
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
            hd = lambda a, b: math.atan2((b.y-a.y), (b.x-a.x))
            

            for i in xrange(self.prev_nrst_wp, self.wp_num):
                
                wp_pos = waypoints.waypoints[i].pose.pose.position

                # distance between vehichle and the nearest waypoint
                dist = dl(self.vehicle_pos, wp_pos)

                if dist < smallest_dist:
                    nearest_wp = i
                    smallest_dist = dist

            # quaternion conversion (see: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion)
            q = self.vehicle_orientation
            theta = math.asin(2*(q.w*q.y + q.z*q.x))
            # if abs(theta)>= 1:
            #     theta = math.pi/2.0
            # else:
            #     theta = math.asin(theta)
            heading =  hd(self.vehicle_pos, wp_pos)
            angle = abs(theta - heading)

            if (angle>math.pi/4.0):
                nearest_wp += 1

            should_brake = self.should_brake()
            final_wps = None
            if not should_brake:
                self.braking = False
                final_wps = self.get_final_wps(waypoints, nearest_wp)
                self.prev_nrst_wp = nearest_wp - 5
                if nearest_wp > 10696:
                    self.prev_nrst_wp = 0

            else:
                self.braking = True
                tl_closest_waypoint_index = self.get_closest_waypoint(self.upcoming_traffic_light_position, waypoints.waypoints)
                final_wps = self.get_final_waypoints(waypoints.waypoints, nearest_wp, tl_closest_waypoint_index)
            
            # rospy.logwarn("nearest waypoint: %s", nearest_wp)
            self.final_waypoints_pub.publish(final_wps)

    def pose_cb(self, msg):
        # TODO: Implement
        # current pose of the vehicle
        
        # Log position change
        if self.vehicle_pos != None:
            distance_change = math.sqrt((msg.pose.position.x - self.vehicle_pos.x)**2 + (msg.pose.position.y - self.vehicle_pos.y)**2)
            if distance_change > 2:
                print("Vehicle position: {}, {}".format(msg.pose.position.x, msg.pose.position.y))
        
        self.vehicle_pos = msg.pose.position

        # vehicle orientation in quanternions
        self.vehicle_orientation = msg.pose.orientation
        # rospy.logwarn("current position of vehicle: %s", msg.pose.position.x)

    def waypoints_cb(self, waypoints):

        self.waypoints = waypoints

    def get_final_wps(self, waypoints, nearest_wp):

        # Collect final waypoints
        final_waypoints = []
        for i in xrange(nearest_wp,nearest_wp+LOOKAHEAD_WPS):
            if i >= self.wp_num:
                break
            final_waypoints.append(waypoints.waypoints[i])

        # Prepare Lane object
        new_wp_lane = Lane()
        new_wp_lane.waypoints = final_waypoints

        return new_wp_lane

    def get_final_waypoints(self, waypoints, start_wp, end_wp):
        final_waypoints = []
        for i in range(start_wp, end_wp):
            index = i % len(waypoints)
            wp = Waypoint()
            wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
            wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
            wp.pose.pose.orientation = waypoints[index].pose.pose.orientation

            if self.braking:
                # Slowly creep up to light if we have stopped short
                dist = self.distance(wp.pose.pose.position, waypoints[end_wp].pose.pose.position)
                if dist > STOP_BUFFER and self.current_linear_velocity < 1.0:
                    wp.twist.twist.linear.x = 2.0
                elif dist < STOP_BUFFER and self.current_linear_velocity < 1.0:
                    wp.twist.twist.linear.x = 0.0
                else:
                    wp.twist.twist.linear.x = min(self.current_linear_velocity, waypoints[index].twist.twist.linear.x)
            else:
                wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
            final_waypoints.append(wp)

        if self.braking:
            # Find the traffic_wp index in final_waypoints to pass to decelerate
            tl_wp = len(final_waypoints)

            # If we are braking set all waypoints passed traffic_wp within LOOKAHEAD_WPS to 0.0
            for i in range(end_wp, start_wp + LOOKAHEAD_WPS):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
                wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                wp.twist.twist.linear.x  = 0.0
                final_waypoints.append(wp)
            final_waypoints = self.decelerate(final_waypoints, tl_wp)

        # Prepare Lane object
        new_wp_lane = Lane()
        new_wp_lane.waypoints = final_waypoints

        return new_wp_lane

    def should_brake(self):
        should_brake = False
        if self.waypoints != None and self.vehicle_pos != None and self.upcoming_traffic_light_position != None:
            wpts = self.waypoints.waypoints
            tl_dist = self.distance(self.vehicle_pos, self.upcoming_traffic_light_position)
            min_stopping_dist = (self.current_linear_velocity**2 / (2.0 * MAX_DECEL) + STOP_BUFFER) * 2
            should_brake = (tl_dist < min_stopping_dist)

        print("Should brake: {}, Has red light: {}".format(should_brake, self.upcoming_traffic_light_position != None))

        return should_brake

    def decelerate(self, waypoints, tl_wp):
        if tl_wp in range(len(waypoints)):
            last = waypoints[tl_wp]
            last.twist.twist.linear.x = 0.0
            for wp in waypoints[:tl_wp][::-1]:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0.0, dist - STOP_BUFFER)
                vel  = math.sqrt(2 * self.decel * dist)
                if vel < 1.0:
                    vel = 0.0
                wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def velocity_cb(self, message):
        self.current_linear_velocity = message.twist.linear.x

    def traffic_cb(self, msg):
        if msg.x != 0 and msg.y != 0:
            self.upcoming_traffic_light_position = msg
        else:
            self.upcoming_traffic_light_position = None

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_closest_waypoint(self, position, waypoints):
        closest_dist = float('inf')
        closest_wp = 0
        for i in range(len(waypoints)):
            dist = self.distance(position, waypoints[i].pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
