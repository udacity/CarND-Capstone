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
import math
import os
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3
SPIN_FREQUENCY = 30

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_list = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config["is_site"])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

    def spin(self, freq):
        """
        Spins this ROS node based on the given frequency.

        :param freq: frequency in hertz.
        """
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if None not in (self.pose, self.waypoints, self.camera_image):
                light_wp, state = self.process_traffic_lights()
                # once process traffic light set camera_image to None so if no image coming we will skip this block
                self.camera_image = None
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
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        # Get the waypoints in X, Y plane and set up the KDTree for efficient comparison.
        self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y]
                             for w in waypoints.waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        for i, stop_line_position in enumerate(stop_line_positions):
            closest_idx = self.waypoints_tree.query([stop_line_position[0], stop_line_position[1]], 1)[1]
            self.stopline_list.append(closest_idx)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        x = pose.position.x
        y = pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Manual Testing
        # return light.state

        # Traffic Light Classifier section
        # If need to manual test, please uncomment Manual Testing block above
        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            for i in range(len(self.stopline_list)):
                if self.stopline_list[i] >= car_position:
                    state = self.get_light_state(self.lights[i])
                    return self.stopline_list[i], state

        #bugbug this line will cause error between last light and end of lap
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector().spin(SPIN_FREQUENCY)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
