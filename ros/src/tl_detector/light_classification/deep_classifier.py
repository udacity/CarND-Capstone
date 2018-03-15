from styx_msgs.msg import TrafficLight
from keras.models import Model
from keras.applications import mobilenet
import numpy as np
import h5py
from keras.models import load_model
import os.path
import scipy.misc as scipy_misc
import tensorflow as tf


class DeepClassifier(object):
    def __init__(self):
        my_path = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(my_path, 'model.h5')

        self.model = load_model(model_path, custom_objects={
                           'relu6': mobilenet.relu6,
                           'DepthwiseConv2D': mobilenet.DepthwiseConv2D})
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_size = (224, 224, 3)
        resized_image = scipy_misc.imresize(image, image_size)
        batch = np.array([resized_image])
        with self.graph.as_default():
            predictions = self.model.predict(batch)
        prediction_softmax = predictions[0]
        prediction = np.argmax(prediction_softmax)
        print(prediction)

        return prediction
