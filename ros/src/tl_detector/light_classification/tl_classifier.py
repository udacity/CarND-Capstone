from styx_msgs.msg import TrafficLight

import numpy as np
import cv2

class TLClassifierSimple(object):
    def __init__(self):
        self.lower = np.array([150, 100, 150])
        self.upper = np.array([180, 255, 255])

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        state = TrafficLight.UNKNOWN

        red_area, red_image = self.get_colored_area(image, self.lower, self.upper)

        # Needs careful tuning for the number of red pixesls
        red_pixels = 40

        if red_area > red_pixels:
            state = TrafficLight.RED

        return state

    def get_colored_area(self, image, lower, upper):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(image, image, mask=mask_image)
        area = cv2.countNonZero(mask_image)

        return area, extracted_image
