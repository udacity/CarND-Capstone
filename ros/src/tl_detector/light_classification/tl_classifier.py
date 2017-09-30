from styx_msgs.msg import TrafficLight
from sim_model import SimModel
from real_model import RealModel
import os

class TLClassifier(object):
    def __init__(self, scenario):
        if scenario == "sim":
            self.model = SimModel()
        else:
            self.model = RealModel("light_classification/models/tl_model")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light, BGR channel

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return self.model.predict(image)
