from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

MINIMUM_NON_ZERO_THRESHOLD=40

class TLClassifier(object):
    def __init__(self):
        pass

    def getColorCount(self, hsv_img, low, high):
        RED_MIN_1 = np.array([low, 100, 100],np.uint8)
        RED_MAX_1 = np.array([high, 255, 255],np.uint8)
        mask = cv2.inRange(hsv_img, RED_MIN_1, RED_MAX_1)
	return cv2.countNonZero(mask)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #Ref: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
        #Red is a unique color in sim env. Easy to recognize. Otherwise it's green (even for yellow case).
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        r1 = self.getColorCount(hsv_img, 0, 10);
        r2 = self.getColorCount(hsv_img, 160, 179);

        return TrafficLight.RED if r1 + r2 > MINIMUM_NON_ZERO_THRESHOLD else TrafficLight.GREEN