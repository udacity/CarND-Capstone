from styx_msgs.msg import TrafficLight
import cv2
import time
import datetime
import random
import img_proc


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
        #ts = time.time()
        #st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d-%H-%M-%S')
        #print('Save image as : ' + st + '.png')
        #cv2.imwrite(st + '.png',image)
        #light color prediction
        int_state  = img_proc.analyze_image(image)

        if int_state == 0:
            return TrafficLight.UNKNOWN
        elif int_state == 1:
            return TrafficLight.RED
        elif int_state == 2:
            return TrafficLight.YELLOW
        elif int_state == 3:
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
