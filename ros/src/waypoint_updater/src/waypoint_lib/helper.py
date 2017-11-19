import math
import tf
from tf import transformations as t


def euclid_dist(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)


def yaw_from_orientation(o):
    # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
    q = (o.x, o.y, o.z, o.w)
    return tf.transformations.euler_from_quaternion(q)[2]


def dist_pose_waypoint(pose, waypoint):
    """Given a pose & a waypoint find the distance between them."""
    dist = euclid_dist(pose.pose.position, waypoint.pose.pose.position)
    return dist


def next_waypoint_index(pose, waypoints):
    """Given a current pose find the closest index of from the list of waypoints.

    :arg:
        pose: Current pose (position) of the vehicle.
        waypoints: The list of waypoints from which to find the closest index
    :returns
        The closest index of the waypoint
    """
    dists = [dist_pose_waypoint(pose, waypoint) for waypoint in waypoints]
    closest_wp_idx = dists.index(min(dists))
    wp = waypoints[closest_wp_idx]

    pose_orientation = pose.pose.orientation

    pose_yaw = yaw_from_orientation(pose_orientation)
    angle = math.atan2(wp.pose.pose.position.y - pose.pose.position.y, wp.pose.pose.position.x - pose.pose.position.x)
    delta = abs(pose_yaw - angle)

    while delta > math.pi:
        delta -= math.pi

    total_waypoints = len(waypoints)

    if delta > math.pi/4:
        closest_wp_idx = (closest_wp_idx + 1) % total_waypoints

    return closest_wp_idx
