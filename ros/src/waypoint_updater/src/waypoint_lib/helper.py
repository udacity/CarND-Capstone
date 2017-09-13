import tf
import math
import rospy

dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

def yaw_from_orientation(o):
    # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
    q = (o.x, o.y, o.z, o.w)
    return tf.transformations.euler_from_quaternion(q)[2]


def dist_pose_waypoint(pose, waypoint):
    return dl(pose.pose.position, waypoint.pose.pose.position)

def closest_waypoint_idx(pose, waypoints):
    dists = [dist_pose_waypoint(pose, wp) for wp in waypoints]
    closest_waypoint = dists.index(min(dists))
    return closest_waypoint

def next_waypoint_idx(pose, waypoints):
    # dists = [dist_pose_waypoint(pose, wp) for wp in waypoints]
    # closest_waypoint = dists.index(min(dists))
    closest_waypoint = closest_waypoint_idx(pose, waypoints)

    waypoints_num = len(waypoints)

    wp = waypoints[closest_waypoint]

    pose_orientation = pose.pose.orientation

    # wp_yaw = helper.yaw_from_orientation(wp_orientation)
    pose_yaw = yaw_from_orientation(pose_orientation)

    angle = math.atan2(wp.pose.pose.position.y-pose.pose.position.y, wp.pose.pose.position.x-pose.pose.position.x)
    # rospy.loginfo('angle1 = {}'.format(angle))
    # rospy.loginfo('pose_yaw = {}'.format(pose_yaw))
    delta = abs(pose_yaw-angle)
    while delta > math.pi: delta -= math.pi
    # rospy.loginfo("delta1 = {}".format(delta))
    if (delta > math.pi/4):
        closest_waypoint = (closest_waypoint + 1) % waypoints_num
        wp = waypoints[closest_waypoint]
        # rospy.loginfo('forward')

    # angle = math.atan2(wp.pose.pose.position.y-pose.pose.position.y, wp.pose.pose.position.x-pose.pose.position.x)
    # delta = abs(pose_yaw-angle)
    # while delta > math.pi: delta -= math.pi

    # rospy.loginfo('angle = {}'.format(angle))
    # rospy.loginfo("delta = {}".format(delta))
    # rospy.loginfo('wp_yaw = {}'.format(wp_yaw))
    return closest_waypoint


def tranform_to_pose_coord_xy(pose, x_coords, y_coords):
    x_coords_pose = []
    y_coords_pose = []
    pose_x = pose.pose.position.x
    pose_y = pose.pose.position.y
    pose_yaw = yaw_from_orientation(pose.pose.orientation)
    for x, y in zip(x_coords, y_coords):
        # Translation
        rx = x - pose_x
        ry = y - pose_y
        # Rotation
        rxf = rx * cos(pose_yaw) + ry * sin(pose_yaw)
        ryf = rx * (-sin(pose_yaw)) + ry * cos(pose_yaw)
        x_coords_pose.append(rxf)
        y_coords_pose.append(ryf)
    return x_coords_pose, y_coords_pose

# moving from wp1 to wp2
def wp_distance(wp1, wp2, waypoints):
    waypoints_num = len(waypoints)
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    curr_wp = wp1
    while curr_wp != wp2:
        next_wp = (curr_wp + 1) % waypoints_num
        dist += dl(waypoints[curr_wp].pose.pose.position, waypoints[next_wp].pose.pose.position)
        curr_wp = next_wp

    # for i in range(wp1, wp2+1):
    #     dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #     wp1 = i

    return dist
