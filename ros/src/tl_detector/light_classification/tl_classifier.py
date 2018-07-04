from styx_msgs.msg import TrafficLight
import cv2
import time
import datetime
import random


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
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d-%H-%M-%S')
        print('Save image as : ' + st + '.png')
        cv2.imwrite(st + '.png',image)


        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
