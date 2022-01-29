#!/usr/bin/env python

"""
This module implements TLDetector.
"""

# ROS imports
import rospy
from scipy.spatial.kdtree import KDTree
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from cv_bridge import CvBridge

# Utils
import tf
import cv2
import yaml
import numpy as np

# Local imports
from light_classification.tl_classifier import TLClassifier

STATE_COUNT_THRESHOLD = 3


class TLDetector:
    """This class implements a traffic light detector."""

    _pose = None  # Cached Ego vehicle pose

    _raw_map_waypoints = None  # Cached map waypoints
    _map_waypoints = None  # Cached map waypoints of type np.ndarray
    _map_waypoints_tree = None  # KDTree for quick searching

    _camera_image = None  # Cached camera image
    _lights = None  # Cached ground truth data from simulator
    _has_image = None  # Has image or not

    def __init__(self):

        self._lights = []

        self._initialize_node()

    def _initialize_node(self):
        rospy.init_node("tl_detector")

        rospy.Subscriber("/current_pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self._map_waypoints_cb)

        # NOTE:
        # `/vehicle/traffic_lights` provides you with the location of the traffic light in 3D.
        # It will help you to acquire an accurate ground truth data source.
        # It contains the current color state of all traffic lights in the simulator.
        # When testing on the vehicle, the color state will not be available. You'll need to
        # rely on the position of the light and the camera image to predict it.
        rospy.Subscriber("/vehicle/traffic_lights", TrafficLightArray, self._traffic_cb)
        rospy.Subscriber("/image_color", Image, self._image_color_cb)
        rospy.Subscriber("/image_raw", Image, self._image_raw_cb)

        self._upcoming_red_light_pub = rospy.Publisher("/traffic_waypoint", Int32, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self._config = yaml.load(config_string)

        self._bridge = CvBridge()
        self._light_classifier = TLClassifier()
        self._listener = tf.TransformListener()

        self._state = TrafficLight.UNKNOWN
        self._prev_state = TrafficLight.UNKNOWN
        self._prev_light_point = -1
        self._state_count = 0

        rospy.spin()

    def _pose_cb(self, msg):
        self._pose = msg

    def _map_waypoints_cb(self, msg):
        """Callback function for receiving map waypoints from ROS topic.

        This function will convert the map waypoints into a KDTree for later quick searching.
        """
        self._raw_map_waypoints = msg
        if not self._map_waypoints:
            self._map_waypoints = np.asarray(
                [
                    [p.pose.pose.position.x, p.pose.pose.position.y]
                    for p in self._raw_map_waypoints.waypoints
                ]
            )
            self._map_waypoints_tree = KDTree(self._map_waypoints)

    def _traffic_cb(self, msg):
        self._lights = msg.lights

    def _image_raw_cb(self, msg):
        pass

    def _image_color_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self._has_image = True
        self._camera_image = msg
        light_point, light_state = self._process_traffic_lights()

        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number # of times
        # till we start using it. Otherwise the previous stable state is used.
        if self._state != light_state:
            self._state_count = 0
            self._state = light_state
        elif self._state_count >= STATE_COUNT_THRESHOLD:
            self._prev_state = self._state
            # Only care about red light.
            light_point = light_point if light_state == TrafficLight.RED else -1
            self._prev_light_point = light_point
            self._upcoming_red_light_pub.publish(Int32(light_point))
        else:
            self._upcoming_red_light_pub.publish(Int32(self._prev_light_point))
        self._state_count += 1

    def _get_nearest_waypoint(self, x, y):
        """Identifies the nearest waypoint on the map to the given position

        Reference: https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        """
        if self._map_waypoints_tree is not None:
            _, nearest_index = self._map_waypoints_tree.query([x, y], k=1)
            return nearest_index
        return -1

    def _get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # This state is coming from simulator
        return light.state

        # TODO: Implement the classifier
        # if self._camera_image is not None:
        #     return self._light_classifier.get_classification(
        #         self._bridge.imgmsg_to_cv2(self._camera_image, "bgr8")
        #     )
        # return TrafficLight.UNKNOWN

    def _process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Each traffic light will be coupled with a stopping line
        nearest_light = None
        stopping_line_index = None

        if self._pose and self._map_waypoints is not None and len(self._map_waypoints):
            x = self._pose.pose.position.x
            y = self._pose.pose.position.y
            ego_waypoint_index = self._get_nearest_waypoint(x, y)

            index_diff = len(self._map_waypoints)

            for light, line in zip(self._lights, self._config["stop_line_positions"]):
                waypoint_index = self._get_nearest_waypoint(line[0], line[1])

                curr_index_diff = waypoint_index - ego_waypoint_index
                if 0 <= curr_index_diff < index_diff:
                    index_diff = curr_index_diff
                    nearest_light = light
                    stopping_line_index = waypoint_index

        if nearest_light:
            light_state = self._get_light_state(nearest_light)
            return stopping_line_index, light_state

        return -1, TrafficLight.UNKNOWN


if __name__ == "__main__":
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start traffic node.")
