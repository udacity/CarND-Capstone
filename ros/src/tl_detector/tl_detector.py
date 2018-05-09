#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import os
import time
import datetime

STATE_COUNT_THRESHOLD = 1
LIGHT_STATES = {0: 'RED', 1: 'YELLOW', 2: 'GREEN', 4: 'UNKNOWN'}

# Distance in meters beyond which we consider the next traffic light is not visible.
TRAFFIC_LIGHT_MAX_DISTANCE = 120

# Time in seconds to wait between consecutive camera captures.
RECORD_SLEEP = 0.05

# Frequency of the main loop in hertz
MAIN_LOOP_FREQUENCY = 5

class TLDetector(object):

    def __init__(self):
        log_level_param = rospy.get_param("/log_level")
        if log_level_param.lower() == 'debug':
            rospy.init_node('tl_detector', log_level=rospy.DEBUG)
        else:
            rospy.init_node('tl_detector')

        self.main_loop_frequency = MAIN_LOOP_FREQUENCY

        # Contains the closest way point for each traffic light stop line
        # It is precomputed as soon as we get the full list of way points, in self.waypoints_cb.
        self.stop_line_wp = []
        self.pose = None
        self.waypoints = None
        self.has_image = False
        self.camera_image = None
        self.lights = []

        # Contains the closest way point for each traffic light stop line
        # It is precomputed as soon as we get the full list of way points, in self.waypoints_cb.
        self.stop_line_wp = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # How will we determine the state of the next traffic light?
        # 'detect': we detect and classify the traffic light on the camera image (not implemented yet).
        # 'oracle': we just use the state provided in topic '/vehicle/traffic_lights' (useful for testing purpose).
        # Default is 'detect', change it in styx.launch.
        self.get_light = rospy.get_param("/get_light").lower()
        self.record_path = rospy.get_param("/record_path")

        # We will save camera captures in folders structured as expected by the fine tuning script retrain.py.
        for subfolder in ['0_red', '1_yellow', '2_green', '4_unknown']:
            path = os.path.join(self.record_path, subfolder)
            if not os.path.exists(path):
                os.makedirs(path)

        self.record_time = time.time()

        # That counter is used to reduce the calling frequency of self.image_cb.
        self.image_cb_counter = -1

        self.upcoming_waypoints = None
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        sub5 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)

        self.loop()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if not (self.waypoints or self.stop_line_wp):
            self.waypoints = waypoints
            self.find_stop_line_closest_waypoints()
            rospy.logdebug(['closest way points to stop lines', self.stop_line_wp])

    def final_waypoints_cb(self, lane):
        self.upcoming_waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Callback function. Receives the image.

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

    def update_state_and_publish(self, light_wp, state):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #rospy.logdebug(['[TLD] State received: ', state])
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def loop(self):
        '''
        Main loop of the Traffic Light Detector Class
        Returns the traffic light "ground truth" if in 'oracle' mode
        or uses a Traffic Light Classifier to detect the state of the
        traffic light on the received image.

        :return: None
        '''
        rate = rospy.Rate(self.main_loop_frequency)
        while not rospy.is_shutdown():
            if ((self.get_light == 'oracle') and (self.pose)) or ((self.get_light == 'detect') and (self.pose) and (self.has_image)):
                light_wp, state = self.process_traffic_lights()
                if self.record_path != '':
                    # Oracle mode and saving images
                    self.save_img(state)
                self.update_state_and_publish(light_wp, state)
            rate.sleep()

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.get_light == 'oracle':
            return light.state
        elif self.get_light == 'detect':
            if self.has_image:
                self.prev_light_loc = None
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
                self.has_image = False
                #Get classification
                return self.light_classifier.get_classification(cv_image)
        return TrafficLight.UNKNOWN

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_id = -1

        # We need to make sure we have the stop line waypoints available
        if not self.stop_line_wp:
            self.find_stop_line_closest_waypoints()
            rospy.logdebug(['closest way points to stop lines', self.stop_line_wp])

        if self.pose and self.stop_line_wp and self.upcoming_waypoints:
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_position = self.upcoming_waypoints[0]
            # The position of the first upcoming wp in the base_waypoints list is given
            # by the waypoint_updater in the header.seq of the first wp.
            car_position_index = car_position.pose.header.seq
            for i, st_wp in enumerate(self.stop_line_wp):
                if st_wp >= car_position_index:
                    light_id = i
                    break

            if st_wp < car_position_index:
                # When the car is beyond the last traffic light, the next traffic light is the first one.
                light_id = 0

            rospy.logdebug(['light_id: ', light_id])
            rospy.logdebug(['size of self.lights[]: ', len(self.lights)])
            light_wp = self.stop_line_wp[light_id]
            light = self.lights[light_id]

            car_p = self.pose.pose.position
            #light_p = light.pose.pose.position
            light_p = self.waypoints.waypoints[light_wp].pose.pose.position
            tl_distance = self.distance(car_p, light_p, 2)

            if tl_distance > TRAFFIC_LIGHT_MAX_DISTANCE:
                # Next traffic light is too far away, we consider it is not visible.
                light = None

            #rospy.logdebug(['car closest way point:', car_position_index])
            #rospy.logdebug(['next stop line closest way point:', light_wp])
            #rospy.logdebug(['car position:', self.pose.pose.position])
            #rospy.logdebug(['next stop line position:', light_p])
            rospy.logdebug(['distance to next traffic light:', tl_distance])

            if light:
                state = self.get_light_state(light)
                if self.get_light != 'oracle':
                    rospy.logdebug(['DETECTED light state:', LIGHT_STATES[state]])
                else:
                    rospy.logdebug(['ORACLE light state:', LIGHT_STATES[light.state]])
                return light_wp, state

        return -1, TrafficLight.UNKNOWN

    def find_stop_line_closest_waypoints(self):
        '''
        Identifies each waypoints which are the last ones before a traffic light.
        Will have to stop at these waypoints if the traffic light is red or yellow.
        :return: None
        '''
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        self.stop_line_wp = []
        for sl_pos in stop_line_positions:
            pose = Pose()
            pose.position.x = sl_pos[0]
            pose.position.y = sl_pos[1]
            pose.position.z = 0
            self.stop_line_wp.append(self.get_closest_waypoint(pose))

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        shortest_distance = np.inf
        closest_wp = -1
        pos = pose.position
        waypoints_list = self.waypoints.waypoints

        for i, wp in enumerate(waypoints_list):
            wp_pos = wp.pose.pose.position
            distance = (wp_pos.x - pos.x) ** 2 + (wp_pos.y - pos.y) ** 2 + (wp_pos.z - pos.z) ** 2
            if distance < shortest_distance:
                closest_wp = i
                shortest_distance = distance
        return closest_wp

    def distance(self, poseA, poseB, dimension):
        '''
        Euclidiean Distance between 2 points in 2 and 3 dimensions
        @note The following poses must have attributes .x, .y for 2 dimensional distance
        and also .z for 3 dimensional distance.
        :param poseA: First Point
        :param poseB: Second Point
        :param dimension: Dimensionality
        :return: Euclidean distance
        '''
        if dimension == 2:
            return np.sqrt((poseA.x - poseB.x) ** 2 + (poseA.y - poseB.y) ** 2)
        elif dimension == 3:
            return np.sqrt((poseA.x - poseB.x) ** 2 + (poseA.y - poseB.y) ** 2 + (poseA.z - poseB.z) ** 2)



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
