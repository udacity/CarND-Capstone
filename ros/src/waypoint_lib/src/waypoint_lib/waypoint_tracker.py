import copy
import math
import rospy

import yaml

import tf as tf_ros                      # This is of ROS geometry, not of TensorFlow!
def get_yaw(orientation):
    """
    Compute yaw from orientation, which is in Quaternion.
    """
    # orientation = msg.pose.orientation
    euler = tf_ros.transformations.euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w])
    yaw = euler[2]
    return yaw
def to_local_coordinates(local_origin_x, local_origin_y, rotation, x, y):
    """
    compute the local coordinates for the global x, y coordinates values,
    given the local_origin_x, local_origin_y, and the rotation of the local x-axis.
    Assume the rotation is radius
    """
    shift_x = x - local_origin_x
    shift_y = y - local_origin_y

    cos_rotation = math.cos(rotation)
    sin_rotation = math.sin(rotation)

    local_x =  cos_rotation*shift_x + sin_rotation*shift_y
    local_y = -sin_rotation*shift_x + cos_rotation*shift_y  # according to John Chen's
    # assuming the orientation angle clockwise being positive
    return local_x, local_y

def waypoint_to_light_f(lights_to_waypoints, base_waypoints_num):
    # implementation
    waypoint_to_light = {}
    light_next = 0

    for waypoint_index in range(base_waypoints_num):
        for light_index in range(light_next, len(lights_to_waypoints)):
            waypoint_index_of_light = lights_to_waypoints[light_index]
            if waypoint_index < waypoint_index_of_light:
                waypoint_to_light[waypoint_index] = (light_index, waypoint_index_of_light)
                break
            elif lights_to_waypoints[-1] <= waypoint_index:
                waypoint_to_light[waypoint_index] = (None, None)
                break
            # end of if waypoint_index <= waypoint_index_of_light
            light_next = light_index
        # end of for light_index in range(len(lights_to_waypoints))
    # end of for i in range(base_waypoints_num)
    return waypoint_to_light

# test data:
lights_to_waypoints = [1, 3, 7, 8, 10, 15]
base_waypoints_num = 17

y = waypoint_to_light_f(lights_to_waypoints, base_waypoints_num)
# expected outcome:
x = (y == {0: (0, 1), 1: (1, 3), 2: (1, 3), 3: (2, 7), 4: (2, 7), 5: (2, 7), 6: (2, 7), 7: (3, 8), 8: (4, 10), 8: (4, 10),
                     9: (4, 10), 10: (5, 15), 11: (5, 15), 12: (5, 15), 13: (5, 15), 14: (5, 15), 15: (None, None), 16: (None, None)})

class WaypointTracker(object):
    def __init__(self):
        self.base_waypoints = None
        self.base_waypoints_num = None
        self.pose = None
        self.lights_to_waypoints = []  # The list of the waypoint index of the traffic lights

        self.last_closest_front_waypoint_index = 0
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # this is a super_class, so it will not start loop nor spin()
        # it expects the subclass will implement the appropriate

    def base_waypoints_process(self, msg):
        # DONE: Implement
        waypoints = msg.waypoints
        if self.base_waypoints is None:
            # unsubscribe to the waypoint messages, no longer needed
            self.base_waypoints_sub.unregister()
            self.subscriber_waypoints = None
    
            self.base_waypoints_num = len(waypoints)
    
            # process the waypoints here
            self.dist_to_here_from_start = []
            self.base_waypoints = []
            dist = 0
            dist_so_far = 0
            self.shortest_dist_to_next_waypoint = 0
            for i in range(self.base_waypoints_num):
                dist_so_far += dist
                self.dist_to_here_from_start.append(dist_so_far)
    
                # do a deep copy of the data, to keep the data from lose
                # just to be safe, simply do shallow copy seems still working
                # by self.base_waypoints = waypoints
                self.base_waypoints.append(copy.deepcopy(waypoints[i]))
                # distance to the next waypoint
                if (i < self.base_waypoints_num-1):
                    dist = (
                        self.distance_two_indices(waypoints,  # the (i+1)_th element has not been copied yet
                                                  i, (i+1) % self.base_waypoints_num))
                # end of if (i < self.base_waypoints_num-1)
                if (dist < self.shortest_dist_to_next_waypoint):
                    self.shortest_dist_to_next_waypoint = dist
                # end of if (dist < self.shortest_dist_to_next_waypoint)
            # end of for i in range(self.base_waypoints_num - 1)
        # end of if self.base_waypoints is None
        # Construct the map, self.waypoint_to_light from a waypoint index to the traffic light
        # in terms of waypoint index
    
        # assumption that a traffic light can only have one waypoint close to it.
        # or one waypoint can have at most one traffic light near it.
        
        # implementation:
        # given a list of coordinates of traffic lights
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        light_cursor = 0
        base_waypoint_search_cursor = 0
        
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        
        # The list of the waypoint index of the traffic lights
        self.lights_to_waypoints = []
        
        for light_cursor in range(len(stop_line_positions)):
            # take, l, the first of the remaining traffic lights coordinates list, self.stop_line_positions
            if base_waypoint_search_cursor < self.base_waypoints_num:
                dist_shortest = dl(self.base_waypoints[base_waypoint_search_cursor].pose.pose.position,
                                    stop_line_positions[light_cursor])
                light_waypoint_index = base_waypoint_search_cursor
        
                # for l to find the closest waypoint in the remaining base_waypoints, w
                for i in range(base_waypoint_search_cursor+1, self.base_waypoints_num):
                    dist = dl(self.base_waypoints[i].pose.pose.position,
                              stop_line_positions[light_cursor])
                    if dist < dist_shortest:
                        dist_shortest = dist
                        light_waypoint_index = i
                    # end of if dist < d_shortest
                # end of for i in range(base_waypoint_search_cursor+1, self.base_waypoints_num)
                # record the mapping from l to w
                self.lights_to_waypoints.append(light_waypoint_index)
                # remove l from the list of traffic lights, and w from the base_points
                base_waypoint_search_cursor = light_waypoint_index + 1
            else:
                # there is extra traffic lights after having found the traffic light for the last waypoint.
                self.lights_to_waypoints.append(None)
            # end of if base_waypoint_search_cursor < self.base_waypoints_num
        # end of for light_cursor in range(len(self.stop_line_positions))
        # until there is no more traffic light, or no more waypoint
        rospy.loginfo('Waypoints for traffic lights: %r' % repr(self.lights_to_waypoints))
        
        # construct the map, self.waypoint_to_light, the map from waypoint index to the index of the
        # traffic light in terms of the closest waypoint index
        self.waypoint_to_light = waypoint_to_light_f(self.lights_to_waypoints, self.base_waypoints_num)
        # rospy.loginfo('test using self.waypoint_to_light[237]: %r' % self.waypoint_to_light[237])
    
        # update self.base_waypoints at light_index to accelerate when there is no red light
        for light_index in self.lights_to_waypoints:
            if light_index is not None:
                # self.base_waypoints[light_index-1].twist.twist.linear.x = (
                #     self.base_waypoints[light_index-1].twist.twist.linear.x * 1.20)
                self.base_waypoints[light_index].twist.twist.linear.x = (
                    self.base_waypoints[light_index].twist.twist.linear.x * 1.10)
            # end of if light_index is not None
        # end of for light_index in self.lights_to_waypoints
    def current_pose_cb(self, msg):
        # WORKING: Implement
        #
        if self.pose is None:       # ready to process message
            self.pose = msg
        # end of if self.pose is None
        # otherwise, the current message is being processed, rejected the coming message and expect to receive more updated next one.
    def get_closest_waypoint(self, pose):
        if self.base_waypoints_num is not None:
            current_pose = pose.position
            current_orientation = pose.orientation
            yaw = get_yaw(current_orientation)
    
            # Compute the waypoints ahead of the current_pose
    
            local_x = -1
            i = self.last_closest_front_waypoint_index - 1
            while ((i < self.base_waypoints_num-1) and (local_x <= 0)):
                i = (i + 1) # % self.base_waypoints_num
                # rospy.loginfo('index of i, searching for the nearest waypoint in front: %r' % i)
                waypoint = self.base_waypoints[i]
                w_pos = waypoint.pose.pose.position
                local_x, local_y = to_local_coordinates(current_pose.x, current_pose.y, yaw,
                                                        w_pos.x, w_pos.y)
            # end of while (local_x < 0)
            self.last_closest_front_waypoint_index = i
            # make the update last_closest_front_waypoint_index atomic with the search of the next one.
            return i
        # end of if self.base_waypoints_num is not None
        return None
    def distance(self, wp1, wp2):
        if (wp1 < wp2):
            start, end = wp1, wp2
        else:
            start, end = wp2, wp1
        # end of if (wp1 < wp2)
    
        dist = self.dist_to_here_from_start[end] - self.dist_to_here_from_start[start]
        return dist
    def distance_two_indices(self, waypoints, i, j):
      a = waypoints[i].pose.pose.position
      b = waypoints[j].pose.pose.position
      return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
