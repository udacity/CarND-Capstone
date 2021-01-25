import cv2
from tl_classifier_cv import detect_red_lights_basic as classify
from tl_classifier_hough import LightDetector
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.light_detector = LightDetector()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        #bgr = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        bgr = image[:,:,::-1]
        images, output_image = self.light_detector.drawCirclesAndGetImages(bgr, True)

        color = self.light_detector.getLightColor(images)
        
        if color == 'red':
            print("color:", color)
            return TrafficLight.RED
        else:
            print("color:", color)
            return TrafficLight.GREEN

        # if classify(image):
        #     return TrafficLight.RED
        # else:
        #     return TrafficLight.GREEN
        #return TrafficLight.UNKNOWN
