from styx_msgs.msg import TrafficLight
from keras.preprocessing.image import load_img, img_to_array
import cv2
import numpy as np
import keras.backend as K
import stopwatch as sw
import tensorflow as tf
from train import SqueezeNet
from consts import IMAGE_WIDTH, IMAGE_HEIGHT

class TLClassifier(object):
    def __init__(self):
        K.set_image_dim_ordering('tf')

        self.ready = False
        self.graph = tf.Graph()
        with self.graph.as_default():
            # Load model from https://github.com/mynameisguy/TrafficLightChallenge-DeepLearning-Nexar
            self.model = SqueezeNet(3, (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
            self.model.load_weights("light_classification/trained_model/challenge1.weights")
            self.model.summary()
            self.ready = True

        self.pred_dict = {0: TrafficLight.UNKNOWN,
                          1: TrafficLight.RED,
                          2: TrafficLight.GREEN}

        self.debug_print = True

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.ready:
            # SqueezeNet model expects RGB image input.
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))
            image = image.astype(K.floatx())
            image /= 255.0
            image = np.expand_dims(image, axis=0)
            with self.graph.as_default():
                preds = self.model.predict(image)[0]
            pred_index = np.argmax(preds)
            pred = self.pred_dict[pred_index]
            if self.debug_print:
                print('TLClassifier', pred, preds)
        else:
            pred = TrafficLight.UNKNOWN

        return pred
