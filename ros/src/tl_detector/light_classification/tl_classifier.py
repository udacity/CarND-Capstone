from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Normalize by number of pixels in image
        num_pixels = image.shape[0] * image.shape[1]

        # Thresholded Red Channel
        _, r_img = cv2.threshold(image[:,:,2], 225, 1, cv2.THRESH_BINARY)
        # Thresholded Green Channel
        _, g_img = cv2.threshold(image[:,:,1], 225, 1, cv2.THRESH_BINARY)
        # Thresholded Yellow Channel
        y_img = r_img * g_img

        if np.sum(y_img.astype(float))/num_pixels > 0.001:
            return TrafficLight.YELLOW
        elif np.sum(g_img.astype(float))/num_pixels > 0.001:
            return TrafficLight.GREEN
        elif np.sum(r_img.astype(float))/num_pixels > 0.001:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
