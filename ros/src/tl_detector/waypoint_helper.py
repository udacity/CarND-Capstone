"""
Helper functions to increase readability in `waypoint_updater.py`
"""


import rospy
import math
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion


def is_ahead(waypoint, car_pose):
    """
    Return `True` if waypoint is ahead of the car.
    """
    car_roll, car_pitch, car_yaw = euler_from_quaternion([car_pose.orientation.x,
                                                          car_pose.orientation.y,
                                                          car_pose.orientation.z,
                                                          car_pose.orientation.w])
    car_x = car_pose.position.x
    car_y = car_pose.position.y

    waypoint_x = waypoint.pose.pose.position.x
    waypoint_y = waypoint.pose.pose.position.y

    shift_x = waypoint_x - car_x
    shift_y = waypoint_y - car_y

    return (shift_x * math.cos(0 - car_yaw) - shift_y * math.sin(0 - car_yaw)) > 0


def get_distance_from_waypoint(waypoint, car_pose):
    """
    Compute euclidean distance between waypoint and the car.
    """
    car_x = car_pose.position.x
    car_y = car_pose.position.y
    car_z = car_pose.position.z

    wp_x = waypoint.pose.pose.position.x
    wp_y = waypoint.pose.pose.position.y
    wp_z = waypoint.pose.pose.position.z

    return math.sqrt((car_x - wp_x) ** 2 + (car_y - wp_y) ** 2 + (car_z - wp_z) ** 2)


def get_simple_distance_from_waypoint(waypoint, car_pose):
    """
    Compute euclidean distance between waypoint and the car.

    This method does not perform the square root function; this optimization 
    is intended for cases where relative distances are compared with other 
    waypoints.
    """
    return (car_pose.position.x - waypoint.pose.pose.position.x) ** 2 + (car_pose.position.y - waypoint.pose.pose.position.y) ** 2


def compose_lane_message(frame_id, waypoints):
    """Helper function for composing the Lane message"""
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane


def print_waypoint(waypoint, msg=''):
    """
    Print coordinates of waypoint (debug purposes)
    """
    wp_x = waypoint.pose.pose.position.x
    wp_y = waypoint.pose.pose.position.y
    wp_z = waypoint.pose.pose.position.z
    rospy.loginfo(msg + ' x: {} y: {} z: {}'.format(wp_x, wp_y, wp_z))
