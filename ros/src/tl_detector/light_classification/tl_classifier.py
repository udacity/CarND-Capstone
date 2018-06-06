from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import os

IMAGE_HEIGHT = 600
IMAGE_WIDTH = 800
IMAGE_CHANNEL = 3

from keras.models import load_model

class TLClassifier(object):
    def __init__(self):
        self.model = None
        self.sim_model_path = None

        #Load configuration
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        #select proper location according to environment
        if not (self.config['tl']['is_carla']):
            self.sim_model_path = './sim_tl_classifier.h5'
        else:
            self.sim_model_path = './real_tl_classifier.h5'

        if self.sim_model_path != None and os.path.exists(self.sim_model_path):
            self.model = load_model(path)
        else:
            # print('Searched for:', self.sim_model_path,'No saved model found!!')
            rospy.logerror('Searched for:', self.sim_model_path,'No saved model found!!')
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        image = cv2.resize(image,(IMAGE_WIDTH, IMAGE_HEIGHT))
        if self.model != None:
            image = np.reshape( image, (1, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNEL))
            scores = self.model.predict(image)
            image_class = np.argmax(scores)
            if image_class == 0:
                return TrafficLight.RED
            elif image_class == 1:
                return TrafficLight.GREEN
            else:
                return TrafficLight.UNKNOWN
        else:
            return TrafficLight.UNKNOWN
