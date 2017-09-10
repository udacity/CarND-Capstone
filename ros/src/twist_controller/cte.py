import numpy as np


class CTE:
    def __init__(self):
        pass

    @staticmethod
    def compute_cte(waypoints, car_pose):
        origin = waypoints[0].pose.pose.position

        # translate waypoints to make origin at (0, 0)
        waypoints_xy = CTE.get_waypoints_xy(waypoints)
        waypoints_xy_origin = waypoints_xy - np.array([origin.x, origin.y])

        # compute angle in radians between the positive x-axis and
        # the point given by the coordinates (x, y)
        offset = 10
        angle = np.arctan2(
            waypoints_xy_origin[offset, 1],  # y
            waypoints_xy_origin[offset, 0])  # x

        # rotate waypoints to align towards positive x-axis
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])

        rotated_waypoints_xy_origin = np.dot(
            waypoints_xy_origin,
            rotation_matrix)

        # fit degree 2 polynomial to waypoints
        degree = 2
        coefficients = CTE.fit_polynomial(
            rotated_waypoints_xy_origin[:, 0],
            rotated_waypoints_xy_origin[:, 1],
            degree)

        # apply origin and rotation transform on car_pose
        car_pose_origin = np.array([
            car_pose.position.x - origin.x,
            car_pose.position.y - origin.y
        ])

        rotated_car_pose_origin = np.dot(car_pose_origin, rotation_matrix)

        # compute trajectory's y coordinate at car x
        car_x, car_y = rotated_car_pose_origin[0], rotated_car_pose_origin[1]
        trajectory_y = CTE.evaluate_polynomial(coefficients, car_x)

        # compute cross-track error
        cte = -(car_y - trajectory_y)
        return cte

    @staticmethod
    def get_waypoints_xy(waypoints):
        points = []

        for waypoint in waypoints:
            points.append([
                waypoint.pose.pose.position.x,
                waypoint.pose.pose.position.y
            ])

        return np.array(points)

    @staticmethod
    def fit_polynomial(x, y, degree):
        coeffs = np.polyfit(x, y, degree)
        return np.flipud(coeffs)  # return lowest degree first

    @staticmethod
    def evaluate_polynomial(coeffs, x):
        y = 0

        for i, coeff in enumerate(coeffs):
            y += coeff * (x ** i)

        return y
