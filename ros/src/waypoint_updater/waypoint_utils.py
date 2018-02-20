"""
Helper functions for waypoints_updater
"""
# pylint: disable=invalid-name

from math import cos, sin, sqrt
from copy import deepcopy

import rospy
import numpy as np
import tf

from styx_msgs.msg import Lane


def construct_lane_object(frame_id, waypoints):
    """ Lane object contains the list of final waypoints ahead with velocity"""
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane


def get_euler(pose):
    """ Returns the roll, pitch yaw angles from a Quaternion \
    Args:
        pose: geometry_msgs/Pose.msg

    Returns:
        roll (float), pitch (float), yaw (float)
    """
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


def get_squared_distance(p1, p2):
    """Returns squared euclidean distance between two 2D points"""
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return dx * dx + dy * dy


def is_waypoint_behind(pose, waypoint):
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose

    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object

    Returns:
        bool : True if the waypoint is behind the car else False

    """
    _, _, yaw = get_euler(pose)
    originX = pose.position.x
    originY = pose.position.y

    shift_x = waypoint.pose.pose.position.x - originX
    shift_y = waypoint.pose.pose.position.y - originY

    x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

    if x > 0:
        return False
    return True


def get_closest_waypoint_index(pose, waypoints):
    """
    pose: geometry_msg.msgs.Pose instance
    waypoints: list of styx_msgs.msg.Waypoint instances
    returns index of the closest waypoint in the list waypoints
    """
    best_gap = float('inf')
    best_index = 0
    my_position = pose.position

    for i, waypoint in enumerate(waypoints):

        other_position = waypoint.pose.pose.position
        gap = get_squared_distance(my_position, other_position)

        if gap < best_gap:
            best_index, best_gap = i, gap

    is_behind = is_waypoint_behind(pose, waypoints[best_index])
    if is_behind:
        best_index += 1
    return best_index


def get_next_waypoints(waypoints, i, n):
    """Return a list of n waypoints ahead of the vehicle"""
    m = min(len(waypoints), i + n)
    return deepcopy(waypoints[i:m])


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """calculates the radius of curvature"""
    if coeffs is None:
        return None
    a = coeffs[0]
    b = coeffs[1]
    return (1 + (2 * a * X + b) ** 2) ** 1.5 / np.absolute(2 * a)
