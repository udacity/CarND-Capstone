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
import copy

STATE_EMA = 0.15                # update parameter using exponential moving average for traffic light state
MAX_DISTANCE_SQ_LIGHT = 10000      # max distance for which we try to detect lights
RED_PROBABILITY_THRESH = 0.5    # consider there is a red light if our confidence is above this threshold

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.light_classifier = None
        self.lights = []

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

        model_path = rospy.get_param("~model_path")
        labels_path = rospy.get_param("~labels_path")
        self.light_classifier = TLClassifier(model_path, labels_path)
        self.listener = tf.TransformListener()

        # Probability of having a red light ahead, updated through EMA
        self.red_state_prob = 0.5

        self.loop()

    def loop(self):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint"""

        rate = rospy.Rate(3)
        while not rospy.is_shutdown():

            # Ensure all required parameters have been initialized
            if all([self.waypoints, self.pose, self.light_classifier, self.camera_image]):

                # get probability of having a red light and its position
                light_wp, red_prob = self.process_traffic_lights()

                # Update total probability of having a red light based on EMA
                self.red_state_prob = STATE_EMA * red_prob + (1 - STATE_EMA) * self.red_state_prob

                # Consider there is no red light if our confidence is low
                if self.red_state_prob < RED_PROBABILITY_THRESH:
                    light_wp = -1

                # Publish upcoming red lights at camera frequency.
                self.upcoming_red_light_pub.publish(Int32(light_wp))            

            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # Ensure we only get initial full list of waypoints as simulator keeps publishing
        # with patial list aftewards
        if self.waypoints is None:
            # We need to get a full copy as otherwise we just get a reference
            self.waypoints = copy.copy(waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        self.camera_image = msg

    def get_car_orientation(self):
        car_x, car_y = self.pose.position.x, self.pose.position.y
        quaternion = (self.pose.orientation.x, self.pose.orientation.y,
                        self.pose.orientation.z, self.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        car_yaw = euler[2]
        return car_yaw
        
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position

        Args:
            pose: position as tuple (x, y)

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_waypoint = min(enumerate(self.waypoints.waypoints),
                               key = lambda wpidx: (wpidx[1].pose.pose.position.x - pose[0]) ** 2
                               + (wpidx[1].pose.pose.position.y - pose[1]) ** 2)
        closest_index = closest_waypoint[0]
        return closest_index

    def closest_light_ahead(self, pos_x, pos_y, yaw, max_distance_light):
        ''' Return position of closest light ahead '''
        
        # Define unit vector for car orientation in global (x, y) coordinates
        orient_x, orient_y = math.cos(yaw), math.sin(yaw)

        # Filter lights to keep only the ones ahead of us by checking scalar product
        lights = self.config['stop_line_positions']
        lights_ahead = [wp for wp in lights if (orient_x * (wp[0] - pos_x) + orient_y * (wp[1] - pos_y)) > 0]
        if not lights_ahead:
            return None
        
        # Extract closest light
        light_dist = [(light, (light[0] - self.pose.position.x) ** 2 + (light[1] - self.pose.position.y) ** 2)
                      for light in lights_ahead]
        closest_light = min(light_dist, key = lambda x: x[1])
        if closest_light[1] > max_distance_light:
            return None

        return closest_light[0]

    def check_red_light(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            float: probability of having a red light

        """
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # Detect traffic light
        red_prob = self.light_classifier.get_classification(cv_image)
        return red_prob

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint close to the upcoming stop line for a traffic light (-1 if none exists)
            float: probability of having a red light ahead
        """
        
        # Get position of closest light ahead
        car_x, car_y = self.pose.position.x, self.pose.position.y
        car_yaw = self.get_car_orientation()
        closest_light = self.closest_light_ahead(car_x, car_y, car_yaw, MAX_DISTANCE_SQ_LIGHT)
        if closest_light is None:
            return -1, 0

        # Verify presence of a red light
        red_prob = self.check_red_light()

        # Get waypoint closest to the light
        light_waypoint_idx = self.get_closest_waypoint(closest_light)

        return light_waypoint_idx, red_prob

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
