import numpy as np


def get_xy_from_waypoints(waypoints):
    """
        Given a list of waypoints, returns a list of [x,y]
        coordinates associated with those waypoints
    """
    return list(map(lambda waypoint: [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y], waypoints))


def get_cross_track_error(final_waypoints, current_pose):
    '''
    The final waypoints are used to fit a polynomial which represents the trajectory
    that the car is expected to take

         (1) The expected y value at the current x value of the car is evaluated by using the curve

         (2) This is compared with the current y value obtained from the car pose
    
    The difference between 1 and 2 is the CTE(Cross Track Error)

    We will transform world coordinates to car coordinates so that all x values are in the
    direction of the car and all y values represent lateral movement
    '''
    origin = final_waypoints[0].pose.pose.position

    waypoints_matrix = get_xy_from_waypoints(final_waypoints)

    # Convert the coordinates [x,y] in the world view to the car's coordinate

    # Shift the points to the origin
    shifted_matrix = waypoints_matrix - np.array([origin.x, origin.y])

    # Derive an angle by which to rotate the points
    offset = 11
    angle = np.arctan2(shifted_matrix[offset, 1], shifted_matrix[offset, 0])
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    rotated_matrix = np.dot(shifted_matrix, rotation_matrix)

    # Fit a 2 degree polynomial to the waypoints
    degree = 2
    coefficients = np.polyfit(
        rotated_matrix[:, 0], rotated_matrix[:, 1], degree)

    # Transform the current pose of the car to be in the car's coordinate system
    shifted_pose = np.array(
        [current_pose.pose.position.x - origin.x, current_pose.pose.position.y - origin.y])
    rotated_pose = np.dot(shifted_pose, rotation_matrix)

    expected_y_value = np.polyval(coefficients, rotated_pose[0])
    actual_y_value = rotated_pose[1]

    return expected_y_value - actual_y_value
