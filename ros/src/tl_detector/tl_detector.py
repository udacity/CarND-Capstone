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
import time
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()

        # Throttle the interval of the TL detection.
        self.min_interval = 0.25
        self.prev_time = time.time()
        self.speed_limit = kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

        self.LOOKAHEAD_WPS = int(self.speed_limit * 10)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.stop_line_waypoints = None

        rospy.spin()

    def throttle_time(self):
        delta_time = time.time() - self.prev_time
        if delta_time >= self.min_interval:
            self.prev_time = time.time()
            return True
        else:
            return False

    def pose_cb(self, msg):
        self.pose = msg.pose

    def get_stop_line_waypoints(self):
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        # get the closest waypoint to each stop_line_position
        slp_waypoints = []
        for slp in stop_line_positions:
            slp_pose = Pose()
            slp_pose.position.x = slp[0]
            slp_pose.position.y = slp[1]
            slp_waypoints.append(self.get_closest_waypoint(slp_pose))

        return slp_waypoints

    def waypoints_cb(self, msg):
        if msg is not None and msg.waypoints is not None:
            self.waypoints = msg.waypoints
            self.stop_line_waypoints = self.get_stop_line_waypoints()

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        if not self.throttle_time():
            return

        self.has_image = True
        self.camera_image = msg

        # Test the inference. Remove this when the rest of the pipeline is built.
        self.infer_camera_image()

        light_wp, state = self.process_traffic_lights()

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def eucl_dist(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        distances = []

        for wp in self.waypoints:
            dist = self.eucl_dist(pose.position, wp.pose.pose.position)
            distances.append(dist)

        index = np.argmin(distances)

        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.has_image:
            self.prev_light_loc = None
            return False

        return self.infer_camera_image()

    def infer_camera_image(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        if self.pose is not None and self.waypoints is not None:
            car_position = self.get_closest_waypoint(self.pose)

        # TODO find the closest visible traffic light (if one exists)
        for i, slw in enumerate(self.stop_line_waypoints):
            if car_position < slw < (car_position + self.LOOKAHEAD_WPS):
                light = self.lights[i]
                light_wp = slw
                break

        if light:
            state = self.get_light_state(light)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN


def kmph2mps(speed_kmph):
    return (speed_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
