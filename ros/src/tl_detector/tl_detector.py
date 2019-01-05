#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial import KDTree
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import os

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Load configuration
        config_string = rospy.get_param("/traffic_light_config")
        
        self.config = yaml.load(config_string)
        self.is_site = self.config['is_site']
        self.current_pose = None
        self.waypoints = None
        self.waypoints_tree = None
        self.stop_line_positions_idx = []
        self.camera_image = None
        self.lights = []
        self.light_state = TrafficLight.UNKNOWN
        self.light_waypoint_idx = -1
        self.light_state_count = 0        
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.light_classifier = None
        # Check if we force the usage of the simulator light state, not available when on site
        if self.config['use_light_state'] and not self.is_site:
            rospy.logwarn('Classifier disabled, using simulator light state')
        else:
            self.light_classifier = TLClassifier()

        # Subscribers
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        # Publisher
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

        if not self.waypoints_tree:
            waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
            self.waypoints_tree = KDTree(waypoints_2d)

            # Precomputes the stop line position index
            stop_line_positions = self.config['stop_line_positions']
            self.stop_line_positions_idx = [self.closest_waypoint_idx(stop_line_position) for stop_line_position in stop_line_positions]
            rospy.logdebug('Stop line positions: %s', stop_line_positions)
            rospy.logdebug('Stop line positions index: %s', self.stop_line_positions_idx)

        # Unsubscribe as we do not need the base waypoints anymore
        self.base_waypoints_sub.unregister()

        rospy.loginfo("Base waypoints data processed, unsubscribed from /base_waypoints")

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        # Image size (sim): 1440000
        light_waypoint_idx, light_state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.light_state != light_state:
            self.light_state_count = 0
            self.light_state = light_state
        elif self.light_state_count >= STATE_COUNT_THRESHOLD:
            light_waypoint_idx = light_waypoint_idx if light_state == TrafficLight.RED else -1
            self.light_waypoint_idx = light_waypoint_idx
            self.upcoming_red_light_pub.publish(Int32(light_waypoint_idx))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.light_waypoint_idx))
        self.light_state_count += 1

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_light_idx = -1
        closest_light_state = TrafficLight.UNKNOWN

        if self.current_pose:
            
            vehicle_position = [self.current_pose.position.x, self.current_pose.position.y]
            vehicle_idx = self.closest_waypoint_idx(vehicle_position)
            min_distance = len(self.waypoints)

            for light, stop_line_idx in zip(self.lights, self.stop_line_positions_idx):
                stop_line_distance = stop_line_idx - vehicle_idx
                if stop_line_distance >= 0 and stop_line_distance < min_distance:
                    min_distance = stop_line_distance
                    closest_light = light
                    closest_light_idx = stop_line_idx

        if closest_light:
            closest_light_state = self.get_light_state(closest_light)

        return closest_light_idx, closest_light_state

    def closest_waypoint_idx(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose: Tuple with x, y coordinates

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.waypoints_tree.query(position, 1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = TrafficLight.UNKNOWN

        # If not classifier is available uses the light state
        if not self.light_classifier:
            light_state = light.state
        elif self.camera_image:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            light_state = self.light_classifier.get_classification(cv_image)

        return light_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
