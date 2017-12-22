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
import os

class TLImageExtractor(object):
    """
    Class extracting images and traffic light positions.

    Subcribed topics:
        /image_color
        /vehicle/traffic_lights
    """

    def __init__(self):
        rospy.init_node('tl_image_extractor')

        self.lights = []
        self.camera_image = None
        self.image_count = 0

        # get parameters
        self.export_directory = rospy.get_param('export_directory', '')
        self.export_filename = rospy.get_param('export_filename', 'tfl_')
        self.export_rate = rospy.get_param('export_rate', '1')
        self.export_encoding = rospy.get_param('export_encoding', 'bgr8')
        self.export_show_image = rospy.get_param('export_show_image', False)

        rospy.loginfo('export_directory: {}'.format(self.export_directory))
        rospy.loginfo('export_filename: {}'.format(self.export_filename))
        rospy.loginfo('export_rate: {}'.format(self.export_rate))
        rospy.loginfo('export_encoding: {}'.format(self.export_encoding))
        rospy.loginfo('export_show_image: {}'.format(self.export_show_image))

        '''
         /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
         helps you acquire an accurate ground truth data source for the traffic light
         classifier by sending the current color state of all traffic lights in the
         simulator. When testing on the vehicle, the color state will not be available. You'll need to
         rely on the position of the light and the camera image to predict it.
         '''
        subscriber_image = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
    
        rospy.spin()

    def traffic_cb(self, msg):
        """Write each received light state and position out to a XML file."""

        self.lights = msg.lights

    def image_cb(self, msg):
        """Writes eah received image to a png file.

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        self.image_count += 1

        if os.path.isdir(self.export_directory) == False:
            rospy.logerr('The export directory "{}" cannot be found.'.format(self.export_directory))
        else:
            filename = "{}/{}{:05d}.png".format(self.export_directory, self.export_filename, self.camera_image.header.seq)
            width = self.camera_image.width
            height = self.camera_image.height
            encoding = self.camera_image.encoding

            # convert ROS to OpenCV image
            if self.image_count % self.export_rate == 0:
                rospy.loginfo('Exported: {} ({:d}x{:d}, {}-->{})'.format(filename, width, height, encoding, self.export_encoding))
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, desired_encoding=self.export_encoding)
                cv2.imwrite(filename, cv_image)

                # show each exported image if activated in launch file
                if self.export_show_image:
                    cv2.imshow('topic: /image_color', cv_image)
                    cv2.waitKey(1)



if __name__ == '__main__':
    try:
        TLImageExtractor()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start image extractor node.')