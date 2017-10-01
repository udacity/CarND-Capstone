#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifier_sim import TLClassifierSim
import tf
import cv2
import yaml
import math
import numpy as np
import threading
import time

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Load config
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site_environment = rospy.get_param('environment', 0)

        # Properties
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = None
        self.state_count = 0
        self.image_lock = threading.RLock()
        self.lights = []
        self.bridge = CvBridge()        
        self.traffic_positions = self.get_given_traffic_lights()

        # Load the classifier
        if self.is_site_environment:
            self.light_classifier = TLClassifier()
        else:
            self.light_classifier = TLClassifierSim()
            
        # Start image processing in recursive loop - one frame at a time
        rospy.Timer(rospy.Duration(0.2), self.process_and_publish, True)

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        if self.is_site_environment:
            rospy.Subscriber('/image_raw', Image, self.image_cb)
        else:
            rospy.Subscriber('/image_color', Image, self.image_cb)

        # Publishers
        self.upcoming_stop_light_pub = rospy.Publisher('/traffic_waypoint', Point, queue_size=1)
        self.recognition_pub = rospy.Publisher('/traffic_light_preview', Image, queue_size=1)

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.image_lock.acquire(True):
            
            self.has_image = True
            self.camera_image = msg

            self.image_lock.release()

    def get_closest_traffic_light_ahead_of_car(self, traffic_lights, car_position, waypoints):
        """
        Given list of traffic lights, car position and waypoints, return closest traffic light
        ahead of the car. This function wraps around the track, so that if car is at the end of the track,
        and closest traffic light is at track's beginning, it will be correctly reported
        :param traffic_lights: list of styx_msgs.msg.TrafficLight instances
        :param car_position: geometry_msgs.msgs.Pose instance
        :param waypoints: list of styx_msgs.msg.Waypoint instances
        :return: styx_msgs.msg.TrafficLight instance
        """

        waypoints_matrix = self.get_waypoints_matrix(waypoints)
        car_index = self.get_closest_waypoint_index(car_position, waypoints_matrix)

        # Arrange track waypoints so they start at car position
        waypoints_ahead = waypoints[car_index:] + waypoints[:car_index]
        waypoints_ahead_matrix = self.get_waypoints_matrix(waypoints_ahead)

        distances = []

        for traffic_light in traffic_lights:

            waypoint_index = self.get_closest_waypoint_index(traffic_light.pose.pose.position, waypoints_ahead_matrix)

            distance = self.get_road_distance(waypoints_ahead[:waypoint_index])
            distances.append(distance)

        closest_traffic_light_index = np.argmin(distances)

        return traffic_lights[closest_traffic_light_index]

    def project_to_image_plane(self, point_in_world, car_pose, image_width, image_height):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world
            car_pose: current car pose
            image_width: camera image width
            image_height: camera image height

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config["camera_info"]["focal_length_x"]
        fy = self.config["camera_info"]["focal_length_y"]

        world_coordinates_point = np.array([point_in_world.x, point_in_world.y, point_in_world.z], dtype=np.float32).reshape(3, 1)

        car_position = np.array([car_pose.position.x, car_pose.position.y, car_pose.position.z], dtype=np.float32).reshape(3, 1)
        camera_offset = np.array([1.0, 0, 1.2], dtype=np.float32).reshape(3, 1)
        translation_vector = car_position + camera_offset

        # Move point to camera origin
        world_coordinates_point_shifted_to_camera_coordinates = world_coordinates_point - translation_vector

        homogenous_vector = np.ones(shape=(4, 1), dtype=np.float32)
        homogenous_vector[:3] = world_coordinates_point_shifted_to_camera_coordinates

        quaternion = np.array([car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w], dtype=np.float32)

        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        rotation_matrix = tf.transformations.euler_matrix(*euler_angles)

        point_in_camera_coordinates = np.dot(rotation_matrix, homogenous_vector)

        x = (fx * point_in_camera_coordinates[0] * point_in_camera_coordinates[2]) + (image_width / 2)
        y = (fy * point_in_camera_coordinates[1] * point_in_camera_coordinates[2]) + (image_height / 2)

        return int(x), int(y)

    def get_waypoints_matrix(self, waypoints):
        """
        Converts waypoints listt to numpy matrix
        :param waypoints: list of styx_msgs.msg.Waypoint instances
        :return: 2D numpy array
        """

        waypoints_matrix = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

        for index, waypoint in enumerate(waypoints):
            waypoints_matrix[index, 0] = waypoint.pose.pose.position.x
            waypoints_matrix[index, 1] = waypoint.pose.pose.position.y

        return waypoints_matrix


    def get_closest_waypoint_index(self, position, waypoints_matrix):
        """
        Given a pose and waypoints list, return index of waypoint closest to pose
        :param position: geometry_msgs.msgs.Position instance
        :param waypoints_matrix: numpy matrix with waypoints coordinates
        :return: integer index
        """

        x_distances = waypoints_matrix[:, 0] - position.x
        y_distances = waypoints_matrix[:, 1] - position.y

        squared_distances = x_distances ** 2 + y_distances ** 2
        return np.argmin(squared_distances)


    def get_road_distance(self, waypoints):
        """
        Get road distance covered when following waypoints
        :param waypoints: list of styx_msgs.msg.Waypoint instances
        :return: float
        """

        total_distance = 0.0

        for index in range(1, len(waypoints)):

            x_distance = waypoints[index].pose.pose.position.x - waypoints[index - 1].pose.pose.position.x
            y_distance = waypoints[index].pose.pose.position.y - waypoints[index - 1].pose.pose.position.y

            distance = np.sqrt((x_distance**2) + (y_distance**2))

            total_distance += distance

        return total_distance

    def get_given_traffic_lights(self):
        """
        Return given traffic light positions
        :return: TrafficLightArray
        """
        traffic_lights = TrafficLightArray()

        traffic_light_list = []

        tl_height = 1.524
        config_string = rospy.get_param("/traffic_light_config")
        traffic_light_positions = yaml.load(config_string)["light_positions"]

        for traffic_light_index, traffic_light_position in enumerate(traffic_light_positions):
            traffic_light = TrafficLight()

            traffic_light.pose.pose.position.x = traffic_light_position[0]
            traffic_light.pose.pose.position.y = traffic_light_position[1]
            traffic_light.pose.pose.position.z = tl_height
            traffic_light.state = TrafficLight.UNKNOWN
            traffic_light_list.append(traffic_light)

            traffic_lights.lights = traffic_light_list

        return traffic_lights

    def get_traffic_light_waypoint(self, pose):

        light_waypoint = None

        car_pose = pose.pose

        if self.traffic_positions != None and car_pose != None and self.waypoints != None and self.camera_image != None:

            traffic_light = self.get_closest_traffic_light_ahead_of_car(self.traffic_positions.lights, car_pose.position, self.waypoints.waypoints)

            light_waypoint = traffic_light

        return light_waypoint

        
    def process_and_publish(self, timer):
        if self.has_image:
            
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''

            position_to_publish = Point() # Empty point because None is causing error

            if self.state != state:
                # Reset counter
                self.state_count = 0
                self.state = state

            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else None
                self.last_wp = light_wp

                if light_wp != None:
                    position_to_publish = light_wp.pose.pose.position

            elif self.last_wp != None:
                position_to_publish = self.last_wp.pose.pose.position
            
            # Publish traffic light Point
            self.upcoming_stop_light_pub.publish(position_to_publish)

            # Increase counter
            self.state_count += 1

        else:

            # Wait for startup if there is no image
            time.sleep(1)

        # Recursive loop - run after
        rospy.Timer(rospy.Duration(0.2), self.process_and_publish, True)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # Default return values
        state = TrafficLight.UNKNOWN   # Default state 
        closest_tf_waypoint = self.get_traffic_light_waypoint(self.pose)

        # Process image if model is ready
        if self.light_classifier.is_loaded is True:

            print("Image process start... {} x {}".format(self.camera_image.width, self.camera_image.height)) 
            state = None
            state_name = None

            # Simulator has different images than rosbags, so we process it differently
            if self.is_site_environment:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
                state, preview_image = self.light_classifier.get_classification(cv_image)
                state_name = self.get_light_name(state)
                # Publish preview
                self.recognition_pub.publish(self.bridge.cv2_to_imgmsg(preview_image, "rgb8"))
            
            else:
                camera_image = self.camera_image
                camera_image.encoding = "rgb8"
                cv_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")
                state, _ = self.light_classifier.get_classification(cv_image)
                state_name = self.get_light_name(state)

            print("Current light: {} {}".format(state, state_name))

        return closest_tf_waypoint, state

    def get_light_name(self, index):
        name = "Unknown"
        if index == TrafficLight.RED:
            name = "RED"
        elif index == TrafficLight.GREEN:
            name = "GREEN"
        elif index == TrafficLight.YELLOW:
            name = "YELLOW"
        return name


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
