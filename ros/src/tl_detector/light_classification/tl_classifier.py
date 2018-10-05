import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # Transform to HSV and simply count the number of color within the range
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # red has hue 0 - 10 & 160 - 180 add another filter
        # TODO  use Guassian mask
        RED_MIN1 = np.array([0, 100, 100],np.uint8)
        RED_MAX1 = np.array([10, 255, 255],np.uint8)

        RED_MIN2 = np.array([160, 100, 100],np.uint8)
        RED_MAX2 = np.array([179, 255, 255],np.uint8)

        frame_threshed1 = cv2.inRange(hsv_img, RED_MIN1, RED_MAX1)
        frame_threshed2 = cv2.inRange(hsv_img, RED_MIN2, RED_MAX2)
        if cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2) > 50:
            return TrafficLight.RED

        YELLOW_MIN = np.array([40.0/360*255, 100, 100],np.uint8)
        YELLOW_MAX = np.array([66.0/360*255, 255, 255],np.uint8)
        frame_threshed3 = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
        if cv2.countNonZero(frame_threshed3) > 50:
            return TrafficLight.YELLOW

        GREEN_MIN = np.array([90.0/360*255, 100, 100],np.uint8)
        GREEN_MAX = np.array([140.0/360*255, 255, 255],np.uint8)
        frame_threshed4 = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)
        if cv2.countNonZero(frame_threshed4) > 50:
            return TrafficLight.GREEN


        return TrafficLight.UNKNOWN
