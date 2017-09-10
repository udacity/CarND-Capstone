import sys
import numpy as np

from styx_msgs.msg import Waypoint

sys.path.append('../src/twist_controller')

from cte import CTE


def test_get_waypoints_xy():
    wp1 = Waypoint()
    wp1.pose.pose.position.x = 1
    wp1.pose.pose.position.y = 2

    wp2 = Waypoint()
    wp2.pose.pose.position.x = 3
    wp2.pose.pose.position.y = 4

    waypoints = [wp1, wp2]

    expected = np.array([
        [1, 2],
        [3, 4],
    ])

    actual = CTE.get_waypoints_xy(waypoints)
    assert (expected == actual).all()


def test_fit_polynomial__straight_line():
    x, y, degree = [0, 5, 10], [7, 7, 7], 1

    expected = [7, 0]
    actual = CTE.fit_polynomial(x, y, degree)

    assert np.allclose(expected, actual)


def test_fit_polynomial__curve():
    x, y, degree = [0, 2, 4], [3, 5, 7], 1

    expected = [3, 1]
    actual = CTE.fit_polynomial(x, y, degree)

    assert np.allclose(expected, actual)


def test_evaluate_polynomial__degree_1():
    coefficients, x = [3.11, 5.7], 1

    expected = 8.81
    actual = CTE.evaluate_polynomial(coefficients, x)

    assert expected == actual


def test_evaluate_polynomial__degree_2():
    coefficients, x = [3.11, 5.7, -2.8], 2

    expected = 3.31
    actual = CTE.evaluate_polynomial(coefficients, x)

    assert np.isclose([expected], actual)
