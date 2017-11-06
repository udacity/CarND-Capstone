#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import time


class TLScanner(object):
    def __init__(self):
        rospy.init_node('tl_scanner')

        self.light_classifier = TLClassifier()

        self.camera_image = None
        self.bridge = CvBridge()

        self.min_interval = 0.25
        self.prev_time = time.time()

        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.spin()

    def image_cb(self, msg):
        delta_time = time.time() - self.prev_time
        if delta_time >= self.min_interval:
            self.prev_time = time.time()
        else:
            return

        self.camera_image = msg
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        self.light_classifier.infer(cv_image)


if __name__ == '__main__':
    try:
        TLScanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
