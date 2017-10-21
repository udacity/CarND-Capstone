#!/usr/bin/env python
import rospy
from image_geometry.cameramodels import PinholeCameraModel
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ab_tl_classify.tl_classifier import TLClassifier
from ab_tl_detect.tl_detection import TLDetection
import tf
import cv2
import PIL
import yaml
import os

from math import *
from scipy import spatial
import numpy as np

STATE_COUNT_THRESHOLD = 1


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.state = None
        self.position = None
        self.orientation = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self._initialized = False
        self.ignore_count = 0

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self._models_initialized = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.sensor_dist = 100
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.stoplineKD = spatial.cKDTree(np.asarray(self.config['stop_line_positions']), leafsize=10)

        self.bridge = CvBridge()

        # Initialize models
        self.traffic_light_detector = TLDetection()
        self.light_classifier = TLClassifier()

        self._models_initialized = True

        self.listener = tf.TransformListener()

        self.camera_model = PinholeCameraModel()

        sub7 = rospy.Subscriber('camera_info', CameraInfo, self.camera_cb, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

        pointsarr = []
        for i in range(len(self.waypoints)):
            pointsarr.append([self.waypoints[i].pose.pose.position.x, self.waypoints[i].pose.pose.position.y])

        # initialize light KD tree
        self.waypointsKD = spatial.cKDTree(np.asarray(pointsarr), leafsize=10)
        self.base_waypoints_sub.unregister()

        self._initialized = True

    def traffic_cb(self, msg):
        self.lights = msg.lights

        lightsarr = []
        for i in range(len(self.lights)):
            lightsarr.append([self.lights[i].pose.pose.position.x, self.lights[i].pose.pose.position.y])

        # initialize light KD tree
        self.lightKD = spatial.cKDTree(np.asarray(lightsarr), leafsize=10)

    def camera_cb(self, msg):
        self.camera_model.fromCameraInfo(msg)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.ignore_count += 1
        if self.ignore_count % 3 != 0:
            return

        self.has_image = True
        self.camera_image = msg

        # rospy.loginfo("[TLDetector] -> processing input image...")
        light_wp, state = self.process_traffic_lights()
        rospy.loginfo("[TLDetector] -> image processed: waypoint= " + str(light_wp) + ", state=" + str(state))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state == TrafficLight.RED or state == TrafficLight.YELLOW:
                light_wp = light_wp
            elif state == TrafficLight.GREEN:
                light_wp = -light_wp
            else:
                light_wp = 1000000

            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, point_x, point_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            point_x: x position to match a waypoint to
            poiny_t: y position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        return self.waypointsKD.query([point_x, point_y], k=1)[1]

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                                           "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                                                         "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # TODO Use tranform and rotation to calculate 2D position of light in image
        point3d = np.array([point_in_world.x, point_in_world.y, point_in_world.z])
        point2d = self.camera_model.project3dToPixel(point3d)

        x, y = point2d

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        result = TrafficLight.UNKNOWN

        if (not self.has_image):
            self.prev_light_loc = None
            return False

        # Don't move until all systems are online
        if (not self._models_initialized):
            return TrafficLight.RED

        # Fix camera encoding to match model (from BGR to RGB)
        if hasattr(self.camera_image, 'encoding'):
            self.attribute = self.camera_image.encoding
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        # Using detection network
        # x, y = self.project_to_image_plane(light.pose.pose.position)

        image = PIL.Image.fromarray(cv_image)

        traffic_lights = self.traffic_light_detector.detect_traffic_lights(image)

        # Get classification
        result = self.light_classifier.get_classification(traffic_lights)

        return result

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if (self._initialized):
            light = None
            light_wp = -1
            light_ahead = self.sensor_dist
            tl_delta = 0.0

            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']

            if (self.position):
                car_position = self.get_closest_waypoint(self.position.x, self.position.y)
                # rospy.loginfo("[TLDetector] -> car position wp index: " + str(car_position))

                car_pose = [self.waypoints[car_position].pose.pose.position.x,
                            self.waypoints[car_position].pose.pose.position.y]
                # rospy.loginfo("[TLDetector] -> car pose: " + str(car_pose))
                tl_delta, light_wp = self.lightKD.query(car_pose, k=1)

                light = self.lights[light_wp]
                # Check negative light locations relative to pose
                cur_q = (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)
                _, _, cur_yaw = tf.transformations.euler_from_quaternion(cur_q)
                light_ahead = ((light.pose.pose.position.x - self.position.x) * cos(cur_yaw) +
                               (light.pose.pose.position.y - self.position.y) * sin(cur_yaw))

                if (tl_delta >= self.sensor_dist or light_ahead < 0.0):
                    light = None

            rospy.loginfo("[TLDetector] -> TL approaching: #" + str(light_wp)
                          + ", ahead: " + str(light_ahead) + "m, distance: " + str(tl_delta))

            if light:

                light_pose = [light.pose.pose.position.x, light.pose.pose.position.y]
                stop_dist, light_wp = self.stoplineKD.query(light_pose, k=1)

                rospy.loginfo("[TLDetector] -> Stop line point in: " + str(stop_dist) + "m")
                rospy.loginfo("[TLDetector] -> Detecting...")

                state = self.get_light_state(light)

                stop_point = [stop_line_positions[light_wp][0], stop_line_positions[light_wp][1]]
                light_wp = self.get_closest_waypoint(stop_point[0], stop_point[1])
                if light_wp < car_position:
                    return -1, TrafficLight.UNKNOWN

                return light_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
