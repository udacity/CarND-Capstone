#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import CameraInfo

from collections import namedtuple

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        # get camera_info like image size from yaml_to_CameraInfo_publisher
        self.num_ch = 3 # number of color channels
        self.Image_Size = namedtuple('image_size', 'x y ch')
        sub_cam_info = rospy.Subscriber('/camera_info_publisher', CameraInfo, self.get_cam_info_cb

    def get_cam_info_cb(self, cam_info):
        """
        """
        self.image_size = self.Image(x=cam_info.height, y=cam_info.width, ch=self.num_ch)

        rospy.logwarn('image_size {}'.format(self.image_size))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
