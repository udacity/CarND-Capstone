import math
import tf
from collections import namedtuple
from styx_msgs.msg import Waypoint

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

    # wrap around
    start = (previous - adjacent) % len(waypoints)
    count = adjacent * 2 + 1
    adjacents = waypoints[start:start + count]
    while len(adjacents) < count:
        adjacents.extend(waypoints[:count - len(adjacents)])

    _, min_dist_i = min_key(adjacents, key=dl)
    closest = (previous - adjacent + min_dist_i) % len(waypoints)
    return closest

def look_ahead_waypoints(pose, waypoints, previous, count):
    """ Only look ahead waypoints in the forward direction """

    waypoint_i = -1
    if previous is not None:
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

def calc_braking_speed(dist, max_decel=1.0):
    vel = math.sqrt(2 * max_decel * dist) * 3.6
    if vel < 1:
        vel = 0.
    return vel

def get_wp_distance(waypoints, wp_first, wp_last):
    """ Get distance between 2 waypoints, wrap around """
    dist = 0

    if wp_first < wp_last:
        for i in range(wp_first, wp_last):
            dist += get_distance(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
    elif wp_last < wp_first:
        for i in range(wp_first, len(waypoints) - 1):
            dist += get_distance(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        dist += get_distance(waypoints[-1].pose.pose.position, waypoints[0].pose.pose.position)
        for i in range(0, wp_last):
            dist += get_distance(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
    return dist

def clone_waypoint(wp):
    p = Waypoint()
    p.pose.pose.position.x = wp.pose.pose.position.x
    p.pose.pose.position.y = wp.pose.pose.position.y
    p.pose.pose.position.z = wp.pose.pose.position.z
    p.pose.pose.orientation.x = wp.pose.pose.orientation.x
    p.pose.pose.orientation.y = wp.pose.pose.orientation.y
    p.pose.pose.orientation.z = wp.pose.pose.orientation.z
    p.pose.pose.orientation.w = wp.pose.pose.orientation.w
    p.twist.twist.linear.x = wp.twist.twist.linear.x
    p.twist.twist.linear.y = wp.twist.twist.linear.y
    p.twist.twist.linear.z = wp.twist.twist.linear.z
    p.twist.twist.angular.x = wp.twist.twist.angular.x
    p.twist.twist.angular.y = wp.twist.twist.angular.y
    p.twist.twist.angular.z = wp.twist.twist.angular.z
    return p

def decelerate_waypoints(base_waypoints,
                         final_waypoints,
                         closest,
                         traffic,
                         stop_distance=5.0,
                         max_decel=1.0):
    decelerated = final_waypoints[:]
    traffic_at = (traffic - closest) % len(base_waypoints)

    # decelerate the waypoints before the traffic light
    next_wp = None
    for i in range(len(decelerated[:traffic_at]))[::-1]:
        wp = decelerated[i]
        if next_wp is None:
            dist_from_traffic = get_wp_distance(base_waypoints, (closest + i) % len(base_waypoints), traffic)
        else:
            dist_from_traffic += get_distance(wp.pose.pose.position, next_wp.pose.pose.position)
        next_wp = wp

        if dist_from_traffic - stop_distance <= 0:
            braking_speed = 0
        else:
            braking_speed = calc_braking_speed(dist_from_traffic - stop_distance, max_decel)
            if braking_speed >= wp.twist.twist.linear.x:
                # no further deceleration needed, break
                break

        new_wp = clone_waypoint(wp)
        new_wp.twist.twist.linear.x = braking_speed
        decelerated[i] = new_wp

    # set the speed to 0 for all waypoints beyond the traffic light
    for i in range(len(decelerated[traffic_at:])):
        wp = decelerated[i]
        new_wp = clone_waypoint(wp)
        new_wp.twist.twist.linear.x = 0
        decelerated[traffic_at + i] = new_wp

    return decelerated
