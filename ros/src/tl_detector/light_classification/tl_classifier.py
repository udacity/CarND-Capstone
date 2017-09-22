from styx_msgs.msg import TrafficLight
from keras.preprocessing.image import img_to_array
import numpy as np
import keras.backend as K
from squeezenet import SqueezeNet
from consts import IMAGE_WIDTH, IMAGE_HEIGHT
import rospy
import cv2

class TLClassifier(object):
    def __init__(self):
        K.set_image_dim_ordering('tf')
        self.model = SqueezeNet(3, (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
        self.model.load_weights("trained_model/challenge1.weights")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.resize(image, (224, 224))
        image = img_to_array(image)
        image /= 255.0
        image = np.expand_dims(image, axis=0)
        preds = self.model.predict(image)[0]
        prediction_result = np.argmax(preds)

        if prediction_result == 0:
            rospy.loginfo('tl_classifier: No traffic light detected.')
            return TrafficLight.UNKNOWN
        elif prediction_result == 1:
            rospy.loginfo('tl_classifier: Red traffic light detected.')
            return TrafficLight.RED
        else:
            rospy.loginfo('tl_classifier: Green traffic light detected')
            return TrafficLight.GREEN