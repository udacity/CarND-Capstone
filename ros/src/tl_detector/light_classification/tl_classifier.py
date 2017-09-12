from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Input, Dense
import tensorflow 

import rospy
import rospkg



class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = None
	self.create_model()

        if  not self.model:
            rospy.logerr("Failed to traffic light classifier model")

        self.colors = [TrafficLight.RED, 
                       TrafficLight.YELLOW,
                       TrafficLight.GREEN,
                       TrafficLight.UNKNOWN]

        self.num_pixels =  950
        
        # Define red pixels in hsv color space
        self.lower_red_1 = np.array([0,  70, 50],   dtype = "uint8")
        self.upper_red_1 = np.array([10, 255, 255], dtype = "uint8")

        self.lower_red_2 = np.array([170,  70,  50], dtype = "uint8")
        self.upper_red_2 = np.array([180, 255, 255], dtype = "uint8")
        

    def create_model(self):
        self.model =  Sequential()
        self.model.add(Dense(200, activation='relu', input_shape=(7800,)))
        self.model.add(Dense(4, activation='softmax'))

        rospack = rospkg.RosPack()
        path_v = rospack.get_path('styx')
        model_file = path_v+ \
               '/../tl_detector/light_classification/tl-classifier-model.h5'
        self.model.load_weights(model_file)
        self.graph = tensorflow.get_default_graph()

    def _get_color_class(self, classes):
	for  i in range(len(classes)):
       	    if classes[i] == 1:
                return self.colors[i]
        return TrafficLight.UNKNOWN

	
    def get_classification_v2(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
   	image_new = image[250:600, 0:1300]
   	dim = (100, 26)
   	resized = cv2.resize(image_new, dim)
   	image_data = np.array([resized.flatten().tolist()])
	with self.graph.as_default():
	     classes = self.model.predict(image_data, batch_size=1)
   	     return self._get_color_class(classes[0])
	return TrafficLight.UNKNOWN
 

    def get_classification(self, image):
        color = TrafficLight.UNKNOWN
        # Convert to hsv space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Mask red pixels
        mask_1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask_2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)

        mask   = cv2.bitwise_or(mask_1, mask_2)

        # Count red pixels
        num_red_pixels = cv2.countNonZero(mask)

        #rospy.loginfo('num_red_pixels: {}'.format(num_red_pixels))

        if num_red_pixels > self.num_pixels:
            color = TrafficLight.RED

        return color
	

    def get_classification_v1(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        color = TrafficLight.UNKNOWN

        # Convert to hsv space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask red pixels
        mask_1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask_2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)

        mask   = cv2.bitwise_or(mask_1, mask_2)

        # Count red pixels
        num_red_pixels = cv2.countNonZero(mask)

        #rospy.loginfo('num_red_pixels: {}'.format(num_red_pixels))

        if num_red_pixels > self.num_pixels:
            color = TrafficLight.RED

        return color
