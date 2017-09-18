from styx_msgs.msg import TrafficLight
from sim_model import SimModel

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = SimModel()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light, BGR channel

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return self.model.predict(image)
