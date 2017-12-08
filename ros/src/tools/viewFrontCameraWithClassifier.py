#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf as tf2
import cv2
import pygame
import sys
import numpy as np
import math
import tensorflow as tf
import yaml
import os

label = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']

class ViewFrontCameraImage():
    def __init__(self, model, camera_topic, config_file):
        # initialize and subscribe to the camera image and traffic lights topic
        rospy.init_node('front_camera_viewer')

        self.model_path = '../classifier/' + model
        self.camera_topic = camera_topic
        self.cv_image = None
        self.lights = []

        with open(os.getcwd()+'/src/tl_detector/'+config_file, 'r') as myconfig:
            config_string=myconfig.read()
            self.tl_config = yaml.load(config_string)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # test different raw image update rates:
        # - 2   - 2 frames a second
        # - 0.5 - 1 frame every two seconds
        self.updateRate = 2

        self.bridge = CvBridge()
        self.camera_sub = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.img_rows = None
        self.img_cols = None
        self.img_ch = None
        self.screen = None
        self.position = None
        self.theta = None
        self.tf_session = None
        self.predict = None

        self.loop()

    def scale(self, x, feature_range=(-1, 1)):
        """Rescale the image pixel values from -1 to 1

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            image (cv::Mat): image rescaled from -1 to 1 pixel values

        """
        # scale to (-1, 1)
        x = ((x - x.min())/(255 - x.min()))

        # scale to feature_range
        min, max = feature_range
        x = x * (max - min) + min
        return x

    def pose_cb(self, msg):
        self.pose = msg
        self.position = self.pose.pose.position
        euler = tf2.transformations.euler_from_quaternion([
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w])
        self.theta = euler[2]

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def dist_to_next_traffic_light(self):
        dist = 100000.
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        ctl = 0
        for i in range(len(self.tl_config['light_positions'])):
            d1 = dl(self.position, self.tl_config['light_positions'][i])
            if dist > d1:
                ctl = i
                dist = d1
        x = self.tl_config['light_positions'][ctl][0]
        y = self.tl_config['light_positions'][ctl][1]
        heading = np.arctan2((y-self.position.y), (x-self.position.x))
        angle = np.abs(self.theta-heading)
        if angle > np.pi/4.:
            ctl += 1
            if ctl >= len(self.tl_config['light_positions']):
                ctl = 0
            dist = dl(self.position, self.tl_config['light_positions'][ctl])
        self.ctl = ctl
        return dist

    def loop(self):
        # only check once a updateRate time in milliseconds...
        font = cv2.FONT_HERSHEY_COMPLEX
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            if self.theta is not None:
                tl_dist = self.dist_to_next_traffic_light()
                if self.camera_sub is None:
                    if tl_dist < 80.:
                        self.camera_sub = rospy.Subscriber(self.camera_topic, Image, self.image_cb)
                    else:
                        if self.img_rows is not None:
                            color = (192, 192, 192)
                            self.cv_image = np.zeros((self.img_rows, self.img_cols, self.img_ch), dtype=np.uint8)
                            text1 = "Nearest Traffic Light (%d)..."
                            text2 = "is %fm ahead."
                            cv2.putText(self.cv_image, text1%(self.ctl), (100, self.img_rows//2-60), font, 1, color, 2)
                            cv2.putText(self.cv_image, text2%(tl_dist), (100, self.img_rows//2), font, 1, color, 2)
                            self.update_pygame()
                else:
                    if tl_dist > 80 and self.camera_sub is not None:
                        self.camera_sub.unregister()
                        self.camera_sub = None
            # schedule next loop
            rate.sleep()

    def update_pygame(self):
        ### initialize pygame
        if self.screen is None:
            self.img_rows = self.cv_image.shape[0]
            self.img_cols = self.cv_image.shape[1]
            self.img_ch = self.cv_image.shape[2]
            pygame.init()
            pygame.display.set_caption("Udacity SDC System Integration Project: Front Camera Viewer")
            self.screen = pygame.display.set_mode((self.img_cols,self.img_rows), pygame.DOUBLEBUF)
        ## give us a machine view of the world
        self.sim_img = pygame.image.fromstring(self.cv_image.tobytes(), (self.img_cols, self.img_rows), 'RGB')
        self.screen.blit(self.sim_img, (0,0))
        pygame.display.flip()

    def image_cb(self, msg):
        """View the incoming camera images as a video

        Args:
            msg (Image): image from car-mounted camera

        """
        # unregister the subscriber to throttle the images coming in
        if self.camera_sub is not None:
            self.camera_sub.unregister()
            self.camera_sub = None

        # set up tensorflow and traffic light classifier
        if self.tf_session is None:
            # get the traffic light classifier
            self.config = tf.ConfigProto(log_device_placement=True)
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.2  # don't hog all the VRAM!
            self.config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
            self.tf_session = tf.Session(config=self.config)
            self.saver = tf.train.import_meta_graph(self.model_path + '/checkpoints/generator.ckpt.meta')
            self.saver.restore(self.tf_session, tf.train.latest_checkpoint(self.model_path + '/checkpoints/'))

            # get the tensors we need for doing the predictions by name
            self.tf_graph = tf.get_default_graph()
            self.input_real = self.tf_graph.get_tensor_by_name("input_real:0")
            self.drop_rate = self.tf_graph.get_tensor_by_name("drop_rate:0")
            self.predict = self.tf_graph.get_tensor_by_name("predict:0")

        if len(self.lights) > 0:
            height = int(msg.height)
            width = int(msg.width)

            # fixing convoluted camera encoding...
            if hasattr(msg, 'encoding'):
                if msg.encoding == '8UC3':
                    msg.encoding = "rgb8"
            else:
                msg.encoding = 'rgb8'

            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            color = (192, 0, 0)
            font = cv2.FONT_HERSHEY_COMPLEX
            text1 = "Nearest Traffic Light (%d)..."
            text2 = "is %fm ahead."
            text3 = 'Most Likely: %s'

            if height != 600 or width != 800:
                image = cv2.resize(self.cv_image, (800, 600), interpolation=cv2.INTER_AREA)
            else:
                image = np.copy(self.cv_image)

            predict = [ TrafficLight.RED ]
            if self.predict is not None:
                predict = self.tf_session.run(self.predict, feed_dict = {
                    self.input_real: self.scale(image.reshape(-1, 600, 800, 3)),
                    self.drop_rate:0.})

            tl_dist = self.dist_to_next_traffic_light()
            cv2.putText(self.cv_image, text1%(self.ctl), (100, height-180), font, 1, color, 2)
            cv2.putText(self.cv_image, text2%(tl_dist), (100, height-120), font, 1, color, 2)
            cv2.putText(self.cv_image, text3%(label[predict[0]]), (100, height-60), font, 1, color, 2)
            self.update_pygame()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Front Camera Image Viewer')
    parser.add_argument('--model', type=str, default='GAN-Semi-Supervised-sim', help='trained GAN model')
    parser.add_argument('--cameratopic', type=str, default='/image_color', help='camera ros topic')
    parser.add_argument('--trafficconfig', type=str, default='sim_traffic_light_config.yaml', help='traffic light yaml config')
    args = parser.parse_args()

    try:
        ViewFrontCameraImage(args.model, args.cameratopic, args.trafficconfig)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start front camera viewer.')


