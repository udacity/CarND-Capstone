from styx_msgs.msg import TrafficLight
from keras.models import load_model
import h5py
import numpy as np
from keras import __version__ as keras_version
import tensorflow as tf
import cv2

class TLClassifier(object):
    def __init__(self,model_fn):

        f = h5py.File(model_fn, mode='r')
        model_version = f.attrs.get('keras_version')
        Keras_version = str(keras_version).encode('utf8')

        if model_version != keras_version:
            rospy.loginfo('You are using Keras version ', keras_version,
                  ', but the model was built using ', model_version)

        self.simulator_model = load_model(model_fn)
        self.simulator_model._make_predict_function()
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        dsize = (200, 150)
        image = cv2.resize(image, dsize)

        image_array = np.asarray(image)

        with self.graph.as_default():
            labels = self.simulator_model.predict(image_array[None,:,:,:])
            predict = np.argmax(labels)

        if predict == 0:
            result = TrafficLight.RED
        elif predict == 1:
            result = TrafficLight.YELLOW
        elif predict == 2:
            result = TrafficLight.GREEN
        else:
            result = TrafficLight.UNKNOWN

        return result
