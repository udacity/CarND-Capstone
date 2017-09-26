# import cv2
# import glob
# from tl_classifier import TLClassifier
#
# light_classifier = TLClassifier('../light_classifier_model.h5')
#
# for files in glob.glob('frame*.jpg'):
#     image = cv2.imread(files)
#     print(light_classifier.get_classification(image))

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tl_classifier import TLClassifier

class TLTest(object):
    def __init__(self):
        self.bridge = CvBridge()
        self._sub = rospy.Subscriber('/image_color', Image, self.callback, queue_size=1)
        self.light_classifier = TLClassifier('../light_classifier_model.h5')
        rospy.spin()

    def callback(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        print(self.light_classifier.get_classification(cv_image))

if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    test = TLTest()
