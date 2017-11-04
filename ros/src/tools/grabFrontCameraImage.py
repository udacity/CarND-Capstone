#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import cv2
import sys
import numpy as np
import yaml
import os

class GrabFrontCameraImage():
    def __init__(self, outfile, camera_topic, config_file):
        # initialize and subscribe to the camera image and traffic lights topic
        rospy.init_node('front_camera_image_grabber')
        self.outfile = outfile

        self.cv_image = None
        self.lights = []
        self.camera_topic = camera_topic

        with open(os.getcwd()+'/src/tl_detector/'+config_file, 'r') as myconfig:
            config_string=myconfig.read()
            self.config = yaml.load(config_string)

        sub2 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub3 = rospy.Subscriber(self.camera_topic, Image, self.image_cb)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        print "trans: ", trans
        print "rot: ", rot
        wp = np.array([ point_in_world.x, point_in_world.y, point_in_world.z ])
        print "point_in_world: ", (wp + trans)
        
        
        x = 0
        y = 0
        return (x, y)

    def draw_light_box(self, light):
        """Draw boxes around traffic lights

        Args:
            light (TrafficLight): light to classify

        Returns:
            image with boxes around traffic lights

        """
        (x,y) = self.project_to_image_plane(light.pose.pose.position)

        # use light location to draw box around traffic light in image
        print "x, y:", x, y


    def image_cb(self, msg):
        """Grab the first incoming camera image and saves it

        Args:
            msg (Image): image from car-mounted camera

        """
        if len(self.lights) > 0:
            height = int(msg.height)
            width = int(msg.width)

            # fixing convoluted camera encoding...
            if hasattr(msg, 'encoding'):
                if msg.encoding == '8UC3':
                    msg.encoding = "rgb8"
            else:
                msg.encoding = 'rgb8'

            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # TODO: experiment with drawing bounding boxes around traffic lights
            # for light in self.lights:
            #     self.draw_light_box(light)

            cv2.imwrite(self.outfile, self.cv_image)
            rospy.signal_shutdown("grabbed image done.")
            sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Front Camera Image Grabber')
    parser.add_argument('--cameratopic', type=str, default='/image_color', help='camera ros topic')
    parser.add_argument('outfile', type=str, help='Image save file')
    parser.add_argument('--trafficconfig', type=str, default='sim_traffic_light_config.yaml', help='traffic light yaml config')
    args = parser.parse_args()

    try:
        GrabFrontCameraImage(args.outfile, args.cameratopic, args.trafficconfig)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not grab front camera image.')


