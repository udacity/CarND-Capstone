from styx_msgs.msg import TrafficLight
from tl_detection.detector import traffic_light_detector

class TLClassifier(object):
    def __init__(self, path):
        #TODO load classifier
        # Load tl detector
        self.tld = traffic_light_detector(path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        box_coords, _ = self.tld.predict(image)

        if len(box_coords):
            return True
        else:
            return False
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
