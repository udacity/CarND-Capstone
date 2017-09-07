import sys

sys.path.append('../src/waypoint_updater')

from geometry_msgs.msg import Point
from load_sim_waypoints import load_sim_waypoints
from waypoint_updater import WaypointUpdater

waypoints = load_sim_waypoints()


def test_closest_waypoint():
    car_point = Point()
    car_point.x = 1131.22
    car_point.y = 1183.27

    closest_index = WaypointUpdater.closest_waypoint(waypoints, car_point)
    assert closest_index == 269


def test_next_waypoint():
    car_point = Point()
    car_point.x = 1131.22
    car_point.y = 1183.27

    next_index = WaypointUpdater.next_waypoint(waypoints, car_point)
    assert next_index == 270
