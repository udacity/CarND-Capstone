#!/usr/bin/env python

import rospy
from tf import transformations
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32
import math
import time
import yaml

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

USE_TRAFFIC_LIGHT    = True # If true, use traffic_lights instead of traffic_waypoint.
WAY_BEFORE_STOP_LINE = 2.75  # Offset [m] to stop line (car position is not measured at front)

LOOKAHEAD_WPS    = 80  # Maximal number of waypoints to publish
MAX_ACCELERATION = 1.0 # Maximal acceleration [m/s^2]
MAX_DECELERATION = 1.0 # Maximal deceleration [m/s^2]


class WaypointUpdater(object):
    
    # TODO: Check init
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

        config_string = rospy.get_param("/traffic_light_config")
        config = yaml.load(config_string)
        self.stop_line_positions = config['stop_line_positions'] # Dictionary with stop line position for each traffic light
        self.lights                    = None # Traffic light positions and states

        #self.time           = 0
        self.last_start_time = None
        self.last_start_time_2 = 0
        self.duration_max = 0;
        self.dt_max = 0;
        self.temp = 0;

        # Run loop.
        self.loop()

    # TODO: Check loop
    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Do not start processing before all data is ready.
            if (self.base_waypoints == None) or (self.pose == None):
                continue

            start_time = end_time_1 = time.time()
            # If traffic light topic is activated, search for red traffic light.
            if (USE_TRAFFIC_LIGHT == True):
                self.search_for_red_traffic_light()
                end_time_1 = time.time()

            if self.prev_base_offset != None:
                start = self.prev_base_offset
                end   = start + 250
            else:
                start = 0
                end   = len(self.base_waypoints)
            next_waypoint_index, _ = self.get_next_waypoint_index(self.pose,
                                         self.base_waypoints[start:end])
            next_waypoint_index += start
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
            speed = lookahead_waypoints[0].twist.twist.linear.x
            # Restore velocities of base waypoints.
            for i in range(len(lookahead_waypoints)):
                self.base_waypoints[start+i].twist.twist.linear.x = original_velocities[i]
            end_time = time.time()
            # Log.
            distance = self.calc_distance_of_points(lookahead_waypoints[0].pose.pose.position, lookahead_waypoints[-1].pose.pose.position)
            if self.last_start_time != None:
                dt = (start_time - self.last_start_time) * 1000.0
                if self.dt_max < dt:
                    self.dt_max = dt
                duration = (end_time - start_time) * 1000.0
                if self.duration_max < duration:
                    self.duration_max = duration
            else:
                dt = 0
            rospy.logwarn('waypoint_updater.py - loop - current_ index: %5i, stop_index: %5i, duration: %6.3f, duration: %6.3f, dt: %6.3f, distance: %6.3f, dt_max: %6.3f, duration_max: %6.3f, len: %3i, speed: %6.3f',
                          self.prev_base_offset, self.waypoint_index_for_stop,
                          (end_time_1 - start_time) * 1000.0,
                          (end_time - end_time_1) * 1000.0,
                          dt,
                          distance,
                          self.dt_max,
                          self.duration_max,
                          len(lookahead_waypoints),
                          speed)
            self.last_start_time = start_time

            rate.sleep()

    def pose_cb(self, msg):
        '''
        Callback for /current_pose message.
        '''
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        '''
        Callback for /base_waypoints message.
        '''
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        '''
        Callback for /traffic_waypoint message.
        '''
        if USE_TRAFFIC_LIGHT == True:
            return
        self.waypoint_index_for_stop = msg.data

    def traffic_test_cb(self, msg):
        '''
        Callback for /traffic_lights message.
        '''
        self.lights = msg.lights

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def calc_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def calc_distance_of_points(self, point_1, point_2):
        '''
        Calculates the distance between two points in 3D.
        '''
        return math.sqrt((point_2.x - point_1.x)**2
                       + (point_2.y - point_1.y)**2
                       + (point_2.z - point_1.z)**2)

    def get_closest_waypoint_index(self, pose, waypoints):
        '''
        Gets index of closest waypoint.
        '''
        closest_distance = 100000 # large number
        closest_index    = 0
        current_position = pose.position
        for i in range(len(waypoints)):
            waypoint_position = waypoints[i].pose.pose.position
            distance = self.calc_distance_of_points(current_position,
                                                    waypoint_position)
            if closest_distance > distance:
                closest_distance = distance
                closest_index    = i
        return closest_index, closest_distance

    def get_next_waypoint_index(self, pose, waypoints):
        '''
        Gets index of next waypoint ahead of the car.
        '''
        # Find closest waypoint.
        closest_wp_index, closest_wp_distance = self.get_closest_waypoint_index(
                                                    pose, waypoints)
        waypoint_position = waypoints[closest_wp_index].pose.pose.position
        current_position  = pose.position
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        _, _, yaw = transformations.euler_from_quaternion(quaternion)
        heading = math.atan2((waypoint_position.y - current_position.y),
                             (waypoint_position.x - current_position.x))
        angle = abs(yaw - heading)
        angle = min(2 * math.pi - angle, angle)
        # Check for angles < 90 degree to get a waypoint ahead of the car.
        if (angle > math.pi/2) and (closest_wp_index < len(waypoints) - 1):
            closest_wp_index += 1
        return closest_wp_index, closest_wp_distance

    # TODO: Check accelerate
    def accelerate(self, waypoints, base_offset):
        '''
        Adjusts the target velocities for the waypoints leading up to top speed
        in order to accelerate the vehicle.
        '''

        # If available, use velocities from previous final waypoints.
        base_offset -= self.prev_base_offset
        if (self.prev_waypoint_velocities != None) and (self.prev_waypoint_velocities[base_offset] > 0.1):
            number_of_previous_points_to_use = 5
            if number_of_previous_points_to_use > (len(self.prev_waypoint_velocities) - base_offset):
                number_of_previous_points_to_use = (len(self.prev_waypoint_velocities) - base_offset)
            for i in range(number_of_previous_points_to_use):
                waypoints[i].twist.twist.linear.x = self.prev_waypoint_velocities[base_offset+i]
        else:
            number_of_previous_points_to_use = 1
            waypoints[0].twist.twist.linear.x = min(1.0, self.base_waypoints[base_offset].twist.twist.linear.x)
        # Accelerate.
        distance_all = 0
        for i in range(number_of_previous_points_to_use, len(waypoints)):
            distance = self.calc_distance_of_points(waypoints[i-1].pose.pose.position,
                                                    waypoints[i].pose.pose.position)
            distance_all += distance
            v_0   = waypoints[i-1].twist.twist.linear.x
            v_max = waypoints[i].twist.twist.linear.x
            p_half = 2 * v_0 / MAX_ACCELERATION / 2
            q = -2 * distance / MAX_ACCELERATION
            dt = -p_half + math.sqrt(p_half * p_half - q)
            velocity = v_0 + MAX_ACCELERATION * dt
            if velocity > v_max - 0.01:
                velocity = v_max
            waypoints[i].twist.twist.linear.x = min(velocity, v_max)
            if distance_all > 25.0:
                waypoints = waypoints[0:i]
                break;
        return waypoints

    # TODO: Check decelerate
    def decelerate(self, waypoints, base_offset):
        '''
        Adjusts the target velocities for the waypoints leading up to red
        traffic lights in order to bring the vehicle to a smooth and full stop.
        '''
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

    def search_for_red_traffic_light(self):
        '''
        Searches for red traffic lights and sets waypoint index for stop.
        '''
        # Reset stop position.
        self.waypoint_index_for_stop = -1
        # If no traffic light data is available, do nothing.
        if not self.lights:
            return

        # Find next traffic light ahead.
        traffic_light_index, distance = self.get_next_waypoint_index(
                                            self.pose, self.lights)
        state = self.lights[traffic_light_index].state
        # Set pose for stop line position.
        stop_line_pose = Pose()
        stop_line_pose.position.x = self.stop_line_positions[traffic_light_index][0]
        stop_line_pose.position.y = self.stop_line_positions[traffic_light_index][1]
        # Check if traffic light is in specified distance and red.
        if (distance < 100.0) and (state == 0):
            # Search waypoint next to stop line.
            start = self.prev_base_offset
            end   = start + 250
            wp_index_next_to_sl, distance = self.get_next_waypoint_index(
                stop_line_pose, self.base_waypoints[start:end])
            wp_index_next_to_sl += start
            # Check for specified distance again, because of reduced search
            # range in base waypoints.
            if distance > 100.0:
                return
            # Search waypoint that is at least a specified distance before the
            # stop line.
            sl_pos = stop_line_pose.position
            for i in range(wp_index_next_to_sl, 0, -1):
                wp_pos = self.base_waypoints[i].pose.pose.position
                distance = self.calc_distance_of_points(wp_pos, sl_pos)
                if distance > WAY_BEFORE_STOP_LINE:
                    # Set stop position.
                    self.waypoint_index_for_stop = i
                    break;

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
