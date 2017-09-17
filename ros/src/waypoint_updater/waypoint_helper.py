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

def get_fwd_bwd_vectors(waypoint_i, waypoints):
    """ Compute vectors for forward and backward direction on the waypoint """
    get_position = lambda wp: wp.pose.pose.position
    vector_fwd = get_vector(get_position(waypoints[waypoint_i]), get_position(waypoints[(waypoint_i + 1) % len(waypoints)]))
    vector_bwd = get_vector(get_position(waypoints[waypoint_i]), get_position(waypoints[(waypoint_i - 1) % len(waypoints)]))
    return vector_fwd, vector_bwd

def get_closest_waypoint(position, waypoints, start=0):
    """ Get the waypoint closest to the pose """
    dl = lambda wp: get_distance(wp.pose.pose.position, position)

    closest_len = float('inf')
    closest_waypoint = start
    next_waypoint = start

    num_waypoints = len(waypoints)
    dist = dl(waypoints[closest_waypoint])

    while (dist < closest_len) and (closest_waypoint < num_waypoints):
        closest_waypoint = next_waypoint
        closest_len = dist
        dist = dl(waypoints[closest_waypoint+1])
        next_waypoint += 1

    return closest_waypoint

def look_ahead_waypoints(pose, waypoints, count=LOOKAHEAD_WPS):
    """ Look ahead waypoints in the direction based on the vehicle current heading """

    waypoint_i = get_closest_waypoint(pose.position, waypoints)
    vector_fwd, vector_bwd = get_fwd_bwd_vectors(waypoint_i, waypoints)

    fwd_yaw = rotations_from_vector(vector_fwd).yaw
    bwd_yaw = rotations_from_vector(vector_bwd).yaw
    current_yaw = rotations_from_quaternion(pose.orientation).yaw

    fwd_yaw_diff = angles_delta(fwd_yaw, current_yaw)
    bwd_yaw_diff = angles_delta(bwd_yaw, current_yaw)

    fwd = True
    if bwd_yaw_diff < fwd_yaw_diff:
        waypoints = waypoints[::-1]
        waypoint_i = len(waypoints) - waypoint_i - 1
        for wp in waypoints:
            wp.twist.twist.linear.x *= -1
        fwd = False

    # Loop to start if waypoints are depleted
    waypoints_ahead = waypoints[waypoint_i:waypoint_i + count]
    while len(waypoints_ahead) < count:
        waypoints_ahead.extend(waypoints[:count - len(waypoints_ahead)])

    # rospy.logout('waypoint_i: %d, yaw %f, fwd_yaw %f, bwd_yaw %f, %s, len(waypoints): %d',
    #        waypoint_i, current_yaw, fwd_yaw, bwd_yaw, 'Forward'  if fwd else 'Backward', len(waypoints_ahead))

    return waypoints_ahead

def look_ahead_waypoints2(self, pose, waypoints, count=LOOKAHEAD_WPS):
    """ Only look ahead waypoints in the forward direction """

    waypoint_i = get_closest_waypoint(pose.position, waypoints)

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

    return waypoints_ahead
