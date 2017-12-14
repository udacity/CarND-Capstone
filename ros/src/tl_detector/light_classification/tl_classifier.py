from styx_msgs.msg import TrafficLight
import label_image
import os.path
import rospy

import tensorflow as tf
import numpy as np
import cv2

MODEL_INPUT_SIZE = 224
MODEL_INPUT_MEAN = 127.5
MODEL_INPUT_STD = 127.5
INPUT_LAYER = "import/input"
OUTPUT_LAYER = "import/final_result"

class TLClassifier(object):

    def __init__(self, model_path, labels_path):

        # Load graph, labels, input and output sensors
        self.graph = label_image.load_graph(model_path)
        self.labels = label_image.load_labels(labels_path)
        self.red_idx = self.get_red_idx()
        self.input_operation = self.graph.get_operation_by_name(INPUT_LAYER)
        self.output_operation = self.graph.get_operation_by_name(OUTPUT_LAYER)

        # Create sessions
        self.sess = tf.Session(graph=self.graph)

    def get_red_idx(self):
        ''' Find index of the red class '''
        
        for n, l in enumerate(self.labels):
            if l == "red":
                return n
        rospy.logerr("'red' was not found as a possible label in 'classifier_labels.txt'")
        raise
        
    def normalize_image(self, image, model_input_size=MODEL_INPUT_SIZE,
                        input_mean=MODEL_INPUT_MEAN, input_std=MODEL_INPUT_STD):
        ''' Ensure we convert the image to a format compatible with the model graph '''
        
        normalized_image = cv2.resize(image, (model_input_size, model_input_size))
        normalized_image = (normalized_image - input_mean) / input_std
        return normalized_image

    def get_classification(self, image):
        """Determines the probability of having a red light

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            float: probability of having a red light
        """

        # Normalize image
        normalized_image = [self.normalize_image(image)]

        # Perform prediction
        results = self.sess.run(self.output_operation.outputs[0],
                           {self.input_operation.outputs[0]: normalized_image})
        results = np.squeeze(results)

        # Probability of having a red light
        return results[self.red_idx]
