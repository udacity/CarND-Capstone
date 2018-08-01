import unittest
import nose

from waypoint_updater import WaypointUpdater

LOOKAHEAD_WPS = 20  # Keep in sync with waypoint_updater.py LOOKAHEAD_WPS
LANE_WAYPOINTS = range(100)


class MyTestCase(unittest.TestCase):
    def generate_lane_start_test(self):
        closest_idx = 0
        wps = WaypointUpdater.calc_base_waypoints(LANE_WAYPOINTS, closest_idx)
        self.assertEqual(range(LOOKAHEAD_WPS), wps)

    def generate_lane_middle_test(self):
        closest_idx = 40
        wps = WaypointUpdater.calc_base_waypoints(LANE_WAYPOINTS, closest_idx)
        self.assertEqual(range(closest_idx, closest_idx + LOOKAHEAD_WPS), wps)

    def generate_lane_end_test(self):
        closest_idx = 80
        wps = WaypointUpdater.calc_base_waypoints(LANE_WAYPOINTS, closest_idx)
        self.assertEqual(range(closest_idx, closest_idx + LOOKAHEAD_WPS), wps)

    def generate_lane_wraparound_test(self):
        closest_idx = 90
        wps = WaypointUpdater.calc_base_waypoints(LANE_WAYPOINTS, closest_idx)
        expected = range(90, 100) + range(0, 10)
        self.assertEqual(expected, wps)

    def test_something(self):
        self.assertEqual(True, True)


if __name__ == '__main__':
    nose.main()
