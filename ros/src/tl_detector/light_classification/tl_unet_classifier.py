from styx_msgs.msg import TrafficLight
import cv2
import tensorflow as tf
import numpy as np

class TLUnetClassifier(object):
    def __init__(self):
        self.model = None
        self.width = 0
        self.height = 0
        self.channels = 3

    def setup_classifier(self, model, width, height, channels=3):
        self.width = width
        self.height = height
        self.model = model
        self.channels = channels
        # necessary work around to avoid troubles with keras
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        resized = cv2.resize(image, (self.width, self.height))
        resized = resized / 255.; # Normalization
        # necessary work around to avoid troubles with keras
        with self.graph.as_default():
            predictions = self.model.predict(resized.reshape((1, self.height, self.width, self.channels)))
            color =  predictions[0].tolist().index(np.max(predictions[0]))
            tl = TrafficLight()
            tl.state = color
            return tl.state
