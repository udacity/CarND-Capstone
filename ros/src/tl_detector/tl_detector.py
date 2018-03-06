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
import numpy as np

STATE_COUNT_THRESHOLD = 3

DEBUG_LEVEL = 2  # 0 no Messages, 1 Important Stuff, 2 Everything
USE_GROUND_TRUTH = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        if DEBUG_LEVEL >= 1: rospy.logwarn("TL Detector loaded!")

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light_wp_prev = None

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
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

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
        self.has_image = True
        self.camera_image = msg
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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # modified version of method by Tobias Wagner in waypoint_updater.py
        # prepare car position and orientation
        car_x = pose.position.x
        car_y = pose.position.y
        s = pose.orientation.w
        car_theta = 2 * np.arccos(s)
        # contain theta between pi and -pi
        if car_theta > np.pi:
            car_theta = -(2 * np.pi - car_theta)
        # a big number to begin with
        mindist = 1000000

        for i in range(len(self.waypoints.waypoints)):
            x = self.waypoints.waypoints[i].pose.pose.position.x
            y = self.waypoints.waypoints[i].pose.pose.position.y

            dist = math.sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y))
            if (dist < mindist):
                mindist = dist
                nwp_x = x
                nwp_y = y
                nwp_index = i

        # this will be the closest waypoint index without respect to heading
        heading = np.arctan2((nwp_y - car_y), (nwp_x - car_x))
        angle = abs(car_theta - heading);
        # so if the heading of the waypoint is over one quarter of pi its behind so take the next wp :)
        if (angle > np.pi / 4):
            nwp_index = (nwp_index + 1) % len(self.waypoints.waypoints)

        return nwp_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(USE_GROUND_TRUTH):
            return self.lights[light].state

        if(not self.has_image):
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
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if (self.pose and self.waypoints):
            car_position = self.get_closest_waypoint(self.pose.pose)

            # Find the waypoint for the next closest road stop line
            for i in range(len(stop_line_positions)):
                stop_line = Pose()
                stop_line.position.x = stop_line_positions[i][0]
                stop_line.position.y = stop_line_positions[i][1]
                light_wp = self.get_closest_waypoint(stop_line)
                if light_wp > car_position:
                    # Road stop line and light coming ahead
                    if light_wp != self.light_wp_prev:
                        # We passed a light; show next one for debugging
                        if DEBUG_LEVEL >= 1:
                            rospy.logwarn("TL Detector car wp: {0:d} pos: {1:.3f},{2:.3f}".format(car_position, self.pose.pose.position.x, self.pose.pose.position.y))
                            rospy.logwarn("TL Detector stop line wp: {0:d} pos: {1:.3f},{2:.3f}".format(light_wp, stop_line.position.x, stop_line.position.y))
                    light = i
                    self.light_wp_prev = light_wp
                    break

            if DEBUG_LEVEL >= 2:
                rospy.logwarn("TL Detector car wp: {0:d} pos: {1:.3f},{2:.3f}".format(car_position, self.pose.pose.position.x, self.pose.pose.position.y))

        if light:
            state = self.get_light_state(light)
            if DEBUG_LEVEL >= 2:
                if self.lights[light].state == TrafficLight.RED:
                    state_name = "RED"
                elif self.lights[light].state == TrafficLight.YELLOW:
                    state_name = "YELLOW"
                elif self.lights[light].state == TrafficLight.GREEN:
                    state_name = "GREEN"
                else:
                    state_name = "UNKNOWN"
                rospy.logwarn("TL Detector stop line wp: {0:d} {1:s}".format(light_wp, state_name))
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
