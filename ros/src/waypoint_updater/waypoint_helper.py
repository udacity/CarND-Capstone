import math
import tf
from collections import namedtuple

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
Rotations = namedtuple('Rotations', ['roll', 'pitch', 'yaw'])
Vector = namedtuple('Vector', ['x', 'y', 'z'])

def get_waypoint_velocity(waypoint):
    return waypoint.twist.twist.linear.x

def set_waypoint_velocity(waypoints, waypoint, velocity):
    waypoints[waypoint].twist.twist.linear.x = velocity

def distance(waypoints, wp1, wp2):
    """ Get distance between 2 waypoints """
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    for i in range(wp1, wp2+1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist

def get_distance(a, b):
    """ Get distance between position A and position B """
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)# + (a.z - b.z) ** 2)

def rotations_from_quaternion(q):
    """ Get rotations on roll, pitch and yaw axes (global) from quaternion """
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[:3]
    return Rotations(roll, pitch, yaw)

def rotations_from_vector(v):
    """ Get rotations on roll, pitch and yaw axes (global) from vector """
    roll = 0
    pitch = math.atan2(v.z, math.sqrt(v.x ** 2 + v.y ** 2))
    yaw = math.atan2(v.y, v.x)
    return Rotations(roll, pitch, yaw)

def get_vector(a, b):
    """ Get vector from position A to position B """
    return Vector(b.x - a.x, b.y - a.y, b.z - a.z)

def angles_delta(angle1, angle2, unit='radian'):
    """ Absolute difference between 2 angles """
    if unit == 'radian':
        return math.pi - math.fabs(math.fabs(angle1 - angle2) - math.pi)
    else:
        return 180 - math.fabs(math.fabs(angle1 - angle2) - 180)

def min_key(array, key=None):
    min_value = float('inf')
    min_i = -1
    if key is not None:
        for i in range(len(array)):
            value = key(array[i])
            if value < min_value:
                min_value = value
                min_i = i
    else:
        for i in range(len(array)):
            value = array[i]
            if value < min_value:
                min_value = value
                min_i = i
    return min_value, min_i

def get_closest_waypoint_from_all(position, waypoints):
    """ Get the waypoint closest to the pose, iterating all waypoints """
    dl = lambda wp: get_distance(wp.pose.pose.position, position)
    _, closest_waypoint = min_key(waypoints, dl)

    # closest_len = float('inf')
    # closest_waypoint = start
    # next_waypoint = start

    # num_waypoints = len(waypoints)
    # dist = dl(waypoints[closest_waypoint])

    # while (closest_waypoint < num_waypoints):
    #     if next_waypoint >= num_waypoints - 1:
    #         break
    #     closest_waypoint = next_waypoint
    #     closest_len = dist
    #     dist = dl(waypoints[closest_waypoint+1])
    #     next_waypoint += 1

    return closest_waypoint

def get_closest_waypoint_from_previous(position, waypoints, previous, adjacent=20):
    """ Get the waypoint closest to the pose based on previous one """
    dl = lambda wp: get_distance(wp.pose.pose.position, position)
    adjacents = waypoints[previous - adjacent: previous + adjacent + 1]
    _, min_dist_i = min_key(adjacents, key=dl)
    closest = previous + min_dist_i - adjacents.index(waypoints[previous])
    return closest

def look_ahead_waypoints(pose, waypoints, previous=None, count=LOOKAHEAD_WPS):
    """ Only look ahead waypoints in the forward direction """

    waypoint_i = -1
    if previous:
        waypoint_i = get_closest_waypoint_from_previous(pose.position, waypoints, previous)
    if waypoint_i == -1:
        waypoint_i = get_closest_waypoint_from_all(pose.position, waypoints)

    current_yaw = rotations_from_quaternion(pose.orientation).yaw

    vector = get_vector(pose.position, waypoints[waypoint_i].pose.pose.position)
    new_rotations = rotations_from_vector(vector)
    new_yaw = new_rotations.yaw

    delta = angles_delta(current_yaw, new_yaw)

    vector_fwd = get_vector(waypoints[waypoint_i].pose.pose.position, waypoints[(waypoint_i + 1) % len(waypoints)].pose.pose.position)
    proposed_yaw = rotations_from_vector(vector_fwd).yaw

    if delta > math.pi / 4:
        waypoint_i = (waypoint_i + 1) % len(waypoints)

    # Loop to start if waypoints are depleted
    waypoints_ahead = waypoints[waypoint_i:waypoint_i + count]
    while len(waypoints_ahead) < count:
        waypoints_ahead.extend(waypoints[:count - len(waypoints_ahead)])

    # rospy.logout('waypoint_i: %d, proposed_yaw %f, len(waypoints): %d', waypoint_i, proposed_yaw, len(waypoints_ahead))

    return waypoints_ahead, waypoint_i
