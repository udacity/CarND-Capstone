from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
#import rospy


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        self.num_pixels = 25
        
        # Define red pixels in hsv color space
        self.lower_red_1 = np.array([0,  70, 50],   dtype = "uint8")
        self.upper_red_1 = np.array([10, 255, 255], dtype = "uint8")

        self.lower_red_2 = np.array([170,  70,  50], dtype = "uint8")
        self.upper_red_2 = np.array([180, 255, 255], dtype = "uint8")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        color = TrafficLight.UNKNOWN

        # Convert to hsv space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask red pixels
        mask_1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask_2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)

        mask   = cv2.bitwise_or(mask_1, mask_2)

        # Count red pixels
        num_red_pixels = cv2.countNonZero(mask)

        #rospy.loginfo('num_red_pixels: {}'.format(num_red_pixels))

        if num_red_pixels > self.num_pixels:
            color = TrafficLight.RED

        return color
