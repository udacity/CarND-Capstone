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
