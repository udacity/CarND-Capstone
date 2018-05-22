"""
Compute The Cross Track Error (CTE)
Adapted from: https://github.com/Kairos-Automotive/carla-brain/blob/master/ros/src/twist_controller/dbw_cte.py
"""

import math
import numpy as np

from tf.transformations import euler_from_quaternion

#WAYPOINTS_LOOKAHEAD = 20 # amount of waypoints for polynomial


def get_cte(waypoints, pose):
    """
    Calculate CTE from waypoint poly and car position
    """
    #coords_x, coords_y = get_points_wrt_pose(waypoints[:WAYPOINTS_LOOKAHEAD], pose)
    coords_x, coords_y = get_points_wrt_pose(waypoints, pose)
    fit = np.poly1d(np.polyfit(coords_x, coords_y, 2))
    cte = fit(2)

    return cte

def get_points_wrt_pose(waypoints, pose):
    """
    Shifts and rotates all waypoints w.r.t to pose orientation.
    Returns a tuple of arrays containing resulting x and y coordinates repectively.
    """
    yaw = yaw_from_orientation(orientation=pose.orientation)

    ref_x, ref_y = pose.position.x, pose.position.y

    shifted_rotated_xs = []
    shifted_rotated_ys = []

    for waypoint in waypoints:
        shift_x = waypoint.pose.pose.position.x - ref_x
        shift_y = waypoint.pose.pose.position.y - ref_y

        shifted_rotated_xs.append(shift_x * math.cos(0 - yaw) - shift_y * math.sin(0 - yaw))
        shifted_rotated_ys.append(shift_x * math.sin(0 - yaw) + shift_y * math.cos(0 - yaw))

    return shifted_rotated_xs, shifted_rotated_ys

def yaw_from_orientation(orientation):
    """
    Computes yaw of orientation
    """
    quaternion = [orientation.x,
                  orientation.y,
                  orientation.z,
                  orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)

    return yaw