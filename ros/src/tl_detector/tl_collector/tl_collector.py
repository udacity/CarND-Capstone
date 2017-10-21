#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import os
import time


class Throttle:
    """
    Throttles the frequency of an action. Set the rate in the constructor, and call
    is_ready(). Throttle will return true every 1/rate calls.
    """

    def __init__(self, rate=3):
        self.rate = rate
        self.count = 0

    def is_ready(self):
        self.count += 1
        if self.count == 1:
            return True
        if self.count > self.rate:
            self.reset()
        return False

    def reset(self):
        self.count = 0


class Regulator:
    """
    Regulates the frequency of the same traffic light class being captured. We want our training set to be balanced
    so it blocks too many red lights being captured in a row.
    """

    def __init__(self, max_per_class=3):
        self.previous_class = -1
        self.class_counter = 0
        self.max_per_class = max_per_class

    def check_class(self, light_class):
        if light_class == self.previous_class:
            self.class_counter += 1
            return self.class_counter <= self.max_per_class
        else:
            self.previous_class = light_class
            self.class_counter = 0
            return True


class TLCollector(object):
    """
    Run this node: rosrun tl_detector tl_collector.
    This node will automatically capture images of traffic lights as we drive past. The images will be sorted
    into folders according to their class, and will be saved in the directory that this node was run from. This node
    will work either with automatic driving or manual driving.
    """
    def __init__(self):
        rospy.init_node('tl_collector')

        # Store information from our subscribed topics.
        self.pose = None
        self.camera_image = None
        self.lights = []

        self.best_light = None
        self.best_light_distance = None
        self.min_capture_range = 20  # m
        self.max_capture_range = 180  # m

        self.bridge = CvBridge()
        self.light_classes = ["red", "yellow", "green", "unknown"]
        self.session_tag = TLCollector.create_session_tag()
        self.create_save_paths()
        self.img_num = 0
        self.cam_throttle = Throttle(rate=8)
        self.regulator = Regulator(max_per_class=5)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        print("Initialized TL Collector")
        rospy.spin()

    def create_save_paths(self):
        for label in self.light_classes:
            path = self.session_tag + "/" + label + "/"
            if not os.path.exists(path):
                os.makedirs(path)

    def pose_cb(self, msg):
        self.pose = msg

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if self.pose is not None:
            self.best_light, self.best_light_distance = self.get_closest_light()

    def image_cb(self, msg):

        self.camera_image = msg

        # Throttle the camera, dropping every X frames (based on rate). We don't need that many pictures.
        if not self.cam_throttle.is_ready():
            return

        if self.best_light is None:
            return

        # Check if the light is within range to be captured.
        if not self.min_capture_range < self.best_light_distance < self.max_capture_range:
            return

        # Do not capture too many of the same light in a row.
        if not self.regulator.check_class(self.best_light.state):
            return

        # All checks passed, capture the image.
        self.capture_image()

    def capture_image(self):

        light_class = self.light_classes[self.best_light.state]
        print("CAPTURE LIGHT: {}".format(light_class))

        cv2_img = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        class_path = self.session_tag + "/" + light_class
        img_path = './{}/sim_{}_{}.jpeg'.format(class_path, light_class, self.img_num)
        cv2.imwrite(img_path, cv2_img)
        self.img_num += 1

    def get_closest_light(self):

        best_distance = float('inf')
        best_light = None
        yaw = self.convert_to_radians(self.pose.pose)
        car_position = self.pose.pose.position

        for light in self.lights:

            # Convert TL to local car co-ordinates.
            lx, ly = self.convert_to_local_space(light.pose, car_position.x, car_position.y, -yaw)

            # Ignore the TL's that we have passed.
            if lx < 0:
                continue

            # Find the TL with the closest distance to us.
            distance = math.sqrt(lx * lx + ly * ly)
            if distance < best_distance:
                best_distance = distance
                best_light = light

        return best_light, best_distance

    @staticmethod
    def create_session_tag():
        session_id = "session_{}".format(time.time())
        return session_id

    @staticmethod
    def convert_to_local_space(pose, tx, ty, yaw_radians):
        nx = pose.pose.position.x - tx
        ny = pose.pose.position.y - ty
        sin_t = math.sin(yaw_radians)
        cos_t = math.cos(yaw_radians)
        fx = nx * cos_t - ny * sin_t
        fy = ny * cos_t + nx * sin_t
        return fx, fy

    @staticmethod
    def convert_to_radians(pose):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        return tf.transformations.euler_from_quaternion(quaternion)[2]


if __name__ == '__main__':
    try:
        TLCollector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic collector node.')
