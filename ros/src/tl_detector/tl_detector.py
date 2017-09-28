#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
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
        self.simulator = True # TODO: add global parameter in the project

        # Properties
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_lock = threading.RLock()
        self.lights = []
        self.bridge = CvBridge()        
        self.light_classifier_loaded = False

        # Load the classifier and change the flag
        if self.simulator:
            self.light_classifier = TLClassifierSim()
            self.light_classifier_loaded = True
            print("Simulator Classifier loaded")
        else:
            self.light_classifier = TLClassifier()
            def start_callback(timer):
                self.light_classifier_loaded = True
                print("TF classifier load delay end")
            rospy.Timer(rospy.Duration(8), start_callback, True) # Wait few seconds until TF model is loaded

        # Start image processing in recursive loop - one frame at a time
        rospy.Timer(rospy.Duration(0.2), self.process_and_publish, True)

        # Subscribers
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        # Publisher
        self.upcoming_stop_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

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


    def get_traffic_light_waypoint(self, pose):
        pass
        
    def process_and_publish(self, timer):
        if self.has_image:
            
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                # Reset counter
                self.state_count = 0
                self.state = state

            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp

                # Publish traffic light data
                self.upcoming_stop_light_pub.publish(Int32(light_wp))

            else:
                # Send last valid value if thereshold is not met
                self.upcoming_stop_light_pub.publish(Int32(self.last_wp))
            
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
        closest_waypoint = -1    

        # Process image if model is ready
        if self.light_classifier_loaded is True:
            print("Image process start...") 
            self.camera_image.encoding = "rgb8"
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            # cv2.imwrite("./../../../asset/test.jpg", cv_image) # debug
            state = self.light_classifier.get_classification(cv_image)
            state_name = self.get_light_name(state)
            print("Current light: {} {}".format(state, state_name))

        return -1, state

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
