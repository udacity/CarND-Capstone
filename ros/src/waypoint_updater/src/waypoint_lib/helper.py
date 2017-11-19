import math


def euclid_dist(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)


def dist_pose_waypoint(pose, waypoint):
    """Given a pose & a waypoint find the distance between them."""
    dist = euclid_dist(pose.pose.position, waypoint.pose.pose.position)
    return dist


def closest_waypoint_index(pose, waypoints):
    """Given a current pose find the closest index of from the list of waypoints.

    :arg:
        pose: Current pose (position) of the vehicle.
        waypoints: The list of waypoints from which to find the closest index
    :returns
        The closest index of the waypoint
    """
    dists = [dist_pose_waypoint(pose, waypoint) for waypoint in waypoints]
    closest_wp_idx = dists.index(min(dists))
    return closest_wp_idx
