#!/usr/bin/env python

import rospy
from tf import transformations
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32
import math
import time

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

USE_TRAFFIC_LIGHT = False # If true, use traffic_lights instead of traffic_waypoint.
LOOKAHEAD_WPS    = 200 # Number of waypoints we will publish. You can change this number.
MAX_ACCELERATION = 1.0 # Maximal acceleration [m/s^2]
MAX_DECELERATION = 1.0 # Maximal deceleration [m/s^2]


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_test_cb, queue_size=1)
        # TODO: Add subscriber for /obstacle_waypoint when implemented.
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints            = None # Waypoint list as it is published for the first time
        self.pose                      = None # Current pose
        self.waypoint_index_for_stop   = -1   # Index for waypoint in base waypoints which is closest to stop line of traffic light
        self.closest_waypoint_index    = None # Index for waypoint in base waypoints which is closest to current position
        self.prev_waypoint_velocities  = None # Velocities of previous final waypoints 
        self.prev_base_offset          = 0    # Base offset of previous final waypoints

        #self.time           = 0
        self.last_start_time = 0
        self.counter         = 0

        # Run loop.
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            if (self.base_waypoints == None) or (self.pose == None):
                continue

            # Check that base waypoints are already received.
            if self.base_waypoints is not None:
                start_time = time.time()
                next_waypoint_index = self.get_next_waypoint_index(self.pose, self.base_waypoints, True)
                # Create a fixed number of waypoints ahead of the car.
                start = next_waypoint_index
                end   = next_waypoint_index + LOOKAHEAD_WPS
                lookahead_waypoints = self.base_waypoints[start:end]
                # Save velocities of base waypoints. To be fast, this saves only
                # values that are possibly changed.
                original_velocities = []
                for i in range(len(lookahead_waypoints)):
                    original_velocities.append(lookahead_waypoints[i].twist.twist.linear.x)

                # TODO: Test acceleration/deceleration.
                #if time.time() > self.time:
                #    if self.waypoint_index_for_stop == -1:
                #        self.waypoint_index_for_stop = next_waypoint_index + 200
                #    else:
                #        self.waypoint_index_for_stop = -1
                #    self.time = time.time() + 25.

                # Accelerate.
                lookahead_waypoints = self.accelerate(lookahead_waypoints, start)
                # Log.
                #rospy.logwarn('waypoint_updater.py - pose_cb - acc: %5.3f',
                #              lookahead_waypoints[0].twist.twist.linear.x)

                # Decelerate.
                lookahead_waypoints = self.decelerate(lookahead_waypoints, start)
                # Log.
                #rospy.logwarn('waypoint_updater.py - pose_cb - ref_vel: %5.3f',
                #              lookahead_waypoints[0].twist.twist.linear.x)

                # Save lookahead waypoint velocities and offset to base waypoints.
                self.prev_waypoint_velocities = []
                for i in range(len(lookahead_waypoints)):
                    self.prev_waypoint_velocities.append(lookahead_waypoints[i].twist.twist.linear.x)
                self.prev_base_offset = start

                # Create the Lane object and fill in waypoints.
                lane                 = Lane()
                lane.waypoints       = lookahead_waypoints
                lane.header.frame_id = '/world'
                lane.header.stamp    = rospy.Time.now()
                # Publish /final_waypoints topic.
                self.final_waypoints_pub.publish(lane)
                # Restore velocities of base waypoints.
                for i in range(len(lookahead_waypoints)):
                    self.base_waypoints[start+i].twist.twist.linear.x = original_velocities[i]
                end_time = time.time()
                # Log.
                rospy.logwarn('waypoint_updater.py - loop - current_ index: %i, stop_index: %i, duration: %5.3f, dt: %5.3f, counter: %i',
                              self.prev_base_offset, self.waypoint_index_for_stop,
                              (end_time - start_time) * 1000.0,
                              (start_time - self.last_start_time) * 1000.0, self.counter)
                self.last_start_time = start_time

            rate.sleep()

    def pose_cb(self, msg):
        # Callback for /current_pose message.
        self.pose = msg.pose
        self.counter += 1

    def waypoints_cb(self, msg):
        # Callback for /base_waypoints message.
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        if USE_TRAFFIC_LIGHT == True:
            return
        self.waypoint_index_for_stop = msg.data

    def traffic_test_cb(self, msg):
        # Callback for /trafficlights message.
        if USE_TRAFFIC_LIGHT == False:
            return
        lights = msg.lights
        if self.base_waypoints is not None:
            state = 0
            if lights:
                # Find next traffic light ahead.
                tl_index = self.get_next_waypoint_index(self.pose, lights, False)
                state = lights[tl_index].state
                if state == 0:
                    waypoint_index_for_light = self.get_next_waypoint_index(lights[tl_index].pose.pose, self.base_waypoints, False)
                    for i in range(waypoint_index_for_light+1, 0, -1):
                        distance = self.calc_distance_of_points(self.base_waypoints[waypoint_index_for_light].pose.pose.position, self.base_waypoints[i].pose.pose.position)
                        if distance > 27.5:
                            self.waypoint_index_for_stop = i
                            break;
                else:
                    self.waypoint_index_for_stop = -1
            else:
                self.waypoint_index_for_stop = -1
                state = -1
            #rospy.logwarn('waypoint_updater.py - traffic_test_cb - state: %i, wp_index: %i',
            #              state, self.waypoint_index_for_stop)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later.
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def calc_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def calc_distance_of_points(self, point_1, point_2):
        # Calculates the distance between two points in 3D.
        return math.sqrt((point_2.x - point_1.x)**2
                       + (point_2.y - point_1.y)**2
                       + (point_2.z - point_1.z)**2)

    def get_closest_waypoint_index(self, pose, waypoints, reduce):
        # Gets index of closest waypoint.
        closest_distance = 100000 # large number
        closest_index    = 0
        current_position = pose.position
        # If closest waypoint was already found, reduce search to its neighborhood.
        if (self.closest_waypoint_index == None) or (reduce == False):
            start = 0
            waypoints_reduced = waypoints
        else:
            start = self.closest_waypoint_index - 10
            waypoints_reduced = waypoints[start:start + 100]
        for index, waypoint in enumerate(waypoints_reduced):
            waypoint_position = waypoint.pose.pose.position
            distance = self.calc_distance_of_points(current_position, waypoint_position)
            if closest_distance > distance:
                closest_distance = distance
                closest_index = index + start
        if (reduce == True):
            self.closest_waypoint_index = closest_index
        return closest_index

    def get_next_waypoint_index(self, pose, waypoints, reduce):
        # Gets next waypoint ahead of the car.
        closest_waypoint_index = self.get_closest_waypoint_index(pose, waypoints, reduce)
        waypoint_position = waypoints[closest_waypoint_index].pose.pose.position
        current_position  = pose.position
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = transformations.euler_from_quaternion(quaternion)
        heading = math.atan2((waypoint_position.y - current_position.y),
                             (waypoint_position.x - current_position.x))
        angle = abs(yaw - heading)
        angle = min(2 * math.pi - angle, angle)
        # Check for angles < 90 degree to get a waypoint ahead of the car.
        if angle > math.pi/2:
            closest_waypoint_index += 1
            if closest_waypoint_index == len(waypoints):
                closest_waypoint_index = 0
        return closest_waypoint_index

    def accelerate(self, waypoints, base_offset):
        # Adjusts the target velocities for the waypoints leading up to top
        # speed in order to accelerate the vehicle.
        # If available, use velocities from previous final waypoints.
        base_offset -= self.prev_base_offset
        if (self.prev_waypoint_velocities != None) and (self.prev_waypoint_velocities[base_offset] > 0.1):
            number_of_previous_points_to_use = 5
            for i in range(number_of_previous_points_to_use):
                waypoints[i].twist.twist.linear.x = self.prev_waypoint_velocities[base_offset+i]
        else:
            number_of_previous_points_to_use = 1
            waypoints[0].twist.twist.linear.x = 1.0
        # Accelerate.
        for i in range(number_of_previous_points_to_use, len(waypoints)):
            distance = self.calc_distance_of_points(waypoints[i-1].pose.pose.position,
                                                    waypoints[i].pose.pose.position)
            v_0   = waypoints[i-1].twist.twist.linear.x
            v_max = waypoints[i].twist.twist.linear.x
            p_half = 2 * v_0 / MAX_ACCELERATION / 2
            q = -2 * distance / MAX_ACCELERATION
            dt = -p_half + math.sqrt(p_half * p_half - q)
            velocity = v_0 + MAX_ACCELERATION * dt
            if velocity > v_max - 0.01:
                velocity = v_max
            waypoints[i].twist.twist.linear.x = min(velocity, v_max)
        return waypoints

    def decelerate(self, waypoints, base_offset):
        # Adjusts the target velocities for the waypoints leading up to red
        # traffic lights in order to bring the vehicle to a smooth and full stop.
        # If stop position is valid, decelerate.
        if self.waypoint_index_for_stop < 0:
            return waypoints
        stop_index = self.waypoint_index_for_stop - base_offset
        if stop_index >= len(waypoints):
            return waypoints
        if stop_index < 0:
            for wp in waypoints:
                wp.twist.twist.linear.x = 0.
            return waypoints
        for wp in waypoints[stop_index:]:
            wp.twist.twist.linear.x = 0.
        stop_wp = waypoints[stop_index]
        for wp in waypoints[:stop_index][::-1]:
            distance = self.calc_distance_of_points(wp.pose.pose.position,
                                                    stop_wp.pose.pose.position)
            velocity = math.sqrt(2 * MAX_DECELERATION * distance)
            if velocity < 1.:
                velocity = 0.
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
