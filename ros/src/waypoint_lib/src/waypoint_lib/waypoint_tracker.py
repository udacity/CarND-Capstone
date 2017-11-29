import copy
import math
import rospy

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

class WaypointTracker(object):
    def __init__(self):
        self.base_waypoints = None
        self.base_waypoints_num = None
        self.pose = None

        self.last_closest_front_waypoint_index = 0
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
    def current_pose_cb(self, msg):
        # WORKING: Implement
        #
        if self.pose is None:       # ready to process message
            self.pose = msg
        # end of if self.pose is None
        # otherwise, the current message is being processed, rejected the coming message and expect to receive more updated next one.
    def get_closest_waypoint(self, pose):
        if self.base_waypoints_num:
            current_pose = pose.position
            current_orientation = pose.orientation
            yaw = get_yaw(current_orientation)
    
            # Compute the waypoints ahead of the current_pose
    
            local_x = -1
            i = self.last_closest_front_waypoint_index - 1
            while ((i < self.base_waypoints_num-1) and (local_x <= 0)):
                i = (i + 1) # % self.base_waypoints_num
                waypoint = self.base_waypoints[i]
                w_pos = waypoint.pose.pose.position
                local_x, local_y = to_local_coordinates(current_pose.x, current_pose.y, yaw,
                                                        w_pos.x, w_pos.y)
                # end of while (local_x < 0)
            return i
        # end of if self.base_waypoints_num
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
