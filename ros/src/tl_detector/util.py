import math
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane

def euclidean_distance(p1x, p1y, p2x, p2y):
    x_dist = p1x - p2x
    y_dist = p1y - p2y
    return math.sqrt(x_dist*x_dist + y_dist*y_dist)

def get_closest_waypoint(pose, waypoints):
    """Identifies the closest path waypoint to the given position
        https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
    Args:
        pose (Pose): position to match a waypoint to

    Returns:
        int: index of the closest waypoint in waypoints

    """

    min_distance = float("inf")
    closest_index = 0
    if waypoints:
        for wp_index in range(len(waypoints.waypoints)):
            waypoint_ps = waypoints.waypoints[wp_index].pose
            distance = euclidean_distance(pose.position.x,
                pose.position.y,
                waypoint_ps.pose.position.x,
                waypoint_ps.pose.position.y)
            if (distance < min_distance):
                min_distance = distance
                closest_index = wp_index

    return closest_index
