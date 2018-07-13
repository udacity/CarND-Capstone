from scipy.spatial import KDTree
import numpy as np


class WaypointSearch(object):
    """Organize waypoints to make it fast to search for the closest to a position"""
    def __init__(self, waypoints):
        # Preprocess waypoints using the k-d tree algorithm
        self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_idx_ahead(self, x, y):
        closest_idx = self.get_closest_waypoint_idx(x, y)

        if not self.__is_closest_idx_ahead(closest_idx, x, y):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def get_closest_waypoint_idx_behind(self, x, y):
        closest_idx = self.get_closest_waypoint_idx(x, y)
        if self.__is_closest_idx_ahead(closest_idx, x, y):
            closest_idx = (closest_idx - 1) % len(self.waypoints_2d)
        return closest_idx

    def get_closest_waypoint_idx(self, x, y):
        return self.waypoints_tree.query([x, y], 1)[1]

    def __is_closest_idx_ahead(self, closest_idx, x, y):
        """ Check if closest_idx is ahead or behind position"""
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        return val < 0
