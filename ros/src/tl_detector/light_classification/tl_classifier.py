from styx_msgs.msg import TrafficLight
import numpy as np


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
        #   TODO implement light color prediction
        # Need to implement traffic light color prediction
        current_light = np.random.choice(
            [TrafficLight.UNKNOWN, TrafficLight.RED,
            TrafficLight.GREEN, TrafficLight.YELLOW]
            )
        return current_light
