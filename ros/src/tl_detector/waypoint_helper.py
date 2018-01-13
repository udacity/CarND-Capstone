import numpy as np
from styx_msgs.msg import TrafficLightArray, TrafficLight
import rospy
import yaml

"""
Helper functions for waypoint dcalculations    
"""


def get_closest_waypoint_index_for_pos(position, waypoints_matrix):
    """
    Returns the index of the closest waypoint to the given position
    :param position: geometry_msgs.msgs.Position instance
    :param waypoints_matrix: numpy matrix with waypoints coordinates
    :return: integer index
    """

    x_distances = waypoints_matrix[:, 0] - position.x
    y_distances = waypoints_matrix[:, 1] - position.y

    squared_distances = x_distances ** 2 + y_distances ** 2
    return np.argmin(squared_distances)

def get_waypoints_matrix(waypoints):
    """
    Converts waypoints list to numpy matrix
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: 2D numpy array
    """

    waypoints_matrix = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

    for index, waypoint in enumerate(waypoints):
        waypoints_matrix[index, 0] = waypoint.pose.pose.position.x
        waypoints_matrix[index, 1] = waypoint.pose.pose.position.y

    return waypoints_matrix


def get_distance_for_waypoints(waypoints):
    """
    Return distance covered by waypoints
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: float
    """

    distance = 0.0

    for index in range(1, len(waypoints)):

        x_distance = waypoints[index].pose.pose.position.x - waypoints[index - 1].pose.pose.position.x
        y_distance = waypoints[index].pose.pose.position.y - waypoints[index - 1].pose.pose.position.y

        distance = np.sqrt((x_distance**2) + (y_distance**2))

    return distance


def get_closest_traffic_light(traffic_lights, car_position, waypoints):
    """
    Given list of traffic lights, car position and waypoints, return closest traffic light
    ahead of the car. This function wraps around the track, so that if car is at the end of the track,
    and closest traffic light is at track's beginning, it will be correctly reported
    :param traffic_lights: list of styx_msgs.msg.TrafficLight instances
    :param car_position: geometry_msgs.msgs.Pose instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: styx_msgs.msg.TrafficLight instance
    """

    waypoints_matrix = get_waypoints_matrix(waypoints)
    car_index = get_closest_waypoint_index_for_pos(car_position, waypoints_matrix)

    waypoints_ahead = waypoints[car_index:] + waypoints[:car_index]
    waypoints_ahead_matrix = get_waypoints_matrix(waypoints_ahead)

    distances = []

    for traffic_light in traffic_lights:

        waypoint_index = get_closest_waypoint_index_for_pos(traffic_light.pose.pose.position, waypoints_ahead_matrix)

        distance = get_distance_for_waypoints(waypoints_ahead[:waypoint_index])
        distances.append(distance)

    closest_traffic_light_index = np.argmin(distances)

    return closest_traffic_light_index, traffic_lights[closest_traffic_light_index]

def get_traffic_lights():
    """
    Returns the trafic light stop positions from config
    :return: TrafficLightArray
    """
    config_string = rospy.get_param("/traffic_light_config")
    traffic_light_positions = yaml.load(config_string)["light_positions"]

    traffic_lights = TrafficLightArray()
    traffic_light_list = []

    for traffic_light_index, traffic_light_position in enumerate(traffic_light_positions):
        traffic_light = TrafficLight()

        traffic_light.pose.pose.position.x = traffic_light_position[0]
        traffic_light.pose.pose.position.y = traffic_light_position[1]
        # TODO: don't know how to get this
        traffic_light.pose.pose.position.z = 0

        traffic_light.state = TrafficLight.UNKNOWN
        traffic_light_list.append(traffic_light)

        traffic_lights.lights = traffic_light_list

    return traffic_lights