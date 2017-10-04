from styx_msgs.msg import TrafficLight
from tl_cnn import TrafficLightModel


class TLClassifierCSR(object):

    def __init__(self):
        self.skip_factor = 0
        self.out_counter = -1
        self.prev_result = TrafficLight.UNKNOWN
        self.model = TrafficLightModel()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.out_counter += 1
        if self.skip_factor > 0 and self.out_counter % self.skip_factor != 0:
            return self.prev_result

        self.prev_result = self.model.predict(image)

        return self.prev_result
