#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import numpy as np
import tf
import cv2
import yaml
from scipy.spatial import KDTree


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # Initialize waypoints structures before callback
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.has_image = False

        # These lists are used for FIR filtering of the detections
        self.last_detections = []
        self.last_detections_w_unknown = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Switch for using live object detection
        self.use_tf_detection = self.config['use_tensorflow_detection']
        self.use_tf_nth_frame = self.config['use_tensorflow_nth_frame']
        self.frame_nb = 0

        self.preprocessing_width = self.config['camera_preprocessing']['image_width']
        self.preprocessing_height = self.config['camera_preprocessing']['image_height']

        self.statecount_threshold = self.config['state_count_threshold']

        # Last detections without UNKNOWN - for FIR filtering
        self.last_detections_size = self.config['detections_filter_size']

        # Last detections with UNKNOWN - for FIR filtering
        self.no_detections_size = self.config['no_detections_filter_size']

        # If in the last detections are more than x percent of UNKNOWN detections,
        # then the current detection is set to UNKNOWN
        self.no_detections_thresh = self.config['no_detections_threshold']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Load the specific Tensorflow graph (site or sim is decided automatically)
        graph_path = self.config['tensorflow_graph_path']
        confidence_thresh = self.config['tensorflow_confidence_thesh']

        # Initialize the classifier
        self.light_classifier = TLClassifier(path_to_tensorflow_graph=graph_path, confidence_thresh=confidence_thresh)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        """Car position updates"""
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """Complete track's waypoint initialization"""
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def state_to_text(self, state):
        return {0: 'red', 1: 'yellow', 2: 'green', 4: 'unknown'}[state]

    def add_traffic_light_detection(self, traffic_light):
        # We manage two lists:
        # One with the UNKNOWN- detections, and one without UNKNOWN
        self.last_detections_w_unknown.append(traffic_light)

        if traffic_light != TrafficLight.UNKNOWN:
            self.last_detections.append(traffic_light)

        # If filter is full, pop first element off
        if len(self.last_detections) > self.last_detections_size:
            self.last_detections.pop(0)

        if len(self.last_detections_w_unknown) > self.no_detections_size:
            self.last_detections_w_unknown.pop(0)

    def get_traffic_light_detection_histogram(self):
        return np.bincount(self.last_detections)

    def get_unknown_rate(self):
        if len(self.last_detections_w_unknown) > 0:
            histogram = np.bincount(self.last_detections_w_unknown)
            histogram = histogram / float(np.sum(histogram))
            if len(histogram) == 5:
                return float(histogram[4])
            return 0.0
        else:
            return 0.0

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # Do not use every frame for classification, but every nth
        self.frame_nb += 1
        if self.frame_nb % self.use_tf_nth_frame == 0:
            self.frame_nb = 0
        else:
            return

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        # Add detection to last detections
        self.add_traffic_light_detection(state)

        # Get the histogram over the last detections
        last_detections_hist = self.get_traffic_light_detection_histogram()

        if len(last_detections_hist) > 0:
            if self.get_unknown_rate() > self.no_detections_thresh:
                # print(self.state_to_text(TrafficLight.UNKNOWN))
                state = TrafficLight.UNKNOWN
            else:
                # print(self.state_to_text(np.argmax(last_detections_hist)))
                state = np.argmax(last_detections_hist)

        readable_state = self.state_to_text(state)

        # Check if there was a change in the detection state
        if self.state != state:
            self.state_count = 0
            self.state = state
            # If there was no change, set if thresholds are exceeded
        else:
            if self.state_count >= self.statecount_threshold:
                if self.last_state != self.state:
                    rospy.loginfo("LiveDetect: {0} | TrafficLight : {1}".format(str(self.use_tf_detection), readable_state))

                self.last_state = self.state
                # SG: I think we should break on Yellow, too
                should_brake = state == TrafficLight.RED or state == TrafficLight.YELLOW
                light_wp = light_wp if should_brake else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x (int): x position to match a waypoint to
            y (int): y position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For testing, just return the light state provided by the simulator. Won't work real-world!!!
        if self.use_tf_detection is False:
            return light.state if light is not None else TrafficLight.UNKNOWN

        if not self.has_image:
            return self.last_state

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        cv_image = cv2.resize(cv_image, (self.preprocessing_width, self.preprocessing_height))
        cv_image = np.expand_dims(cv_image, 0)

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            # Finds the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index ahead
                d = temp_wp_idx - car_wp_idx
                if 0 <= d < diff:
                    # TODO check if also d < max-distance-to-brake-safely to avoid computing get_light_state()
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        state = self.get_light_state(closest_light)
        return line_wp_idx, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
