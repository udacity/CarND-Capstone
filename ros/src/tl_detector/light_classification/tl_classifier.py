from styx_msgs.msg import TrafficLight
import label_image
import os.path

import tensorflow as tf
import numpy as np
import cv2

MODEL_FILE = "classifier_graph_sim.pb"
LABEL_FILE = "classifier_labels.txt"
MODEL_INPUT_SIZE = 224
MODEL_INPUT_MEAN = 127.5
MODEL_INPUT_STD = 127.5
INPUT_LAYER = "import/input"
OUTPUT_LAYER = "import/final_result"

class TLClassifier(object):

    def __init__(self):

        # Load graph, labels, input and output sensors
        model_path = os.path.join(os.path.dirname(__file__), MODEL_FILE)
        label_path = os.path.join(os.path.dirname(__file__), LABEL_FILE)
        self.graph = label_image.load_graph(model_path)
        self.labels = label_image.load_labels(label_path)
        self.input_operation = self.graph.get_operation_by_name(INPUT_LAYER)
        self.output_operation = self.graph.get_operation_by_name(OUTPUT_LAYER)

        # Create sessions
        self.sess = tf.Session(graph=self.graph)

    def normalize_image(self, image, model_input_size=MODEL_INPUT_SIZE,
			            input_mean=MODEL_INPUT_MEAN, input_std=MODEL_INPUT_STD):
        ''' Ensure we convert the image to a format compatible with the model graph '''
        
        normalized_image = cv2.resize(image, (model_input_size, model_input_size))
        normalized_image = (normalized_image - input_mean) / input_std
        return normalized_image

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # Normalize image
        normalized_image = [self.normalize_image(image)]

        # Perform prediction
        results = self.sess.run(self.output_operation.outputs[0],
                           {self.input_operation.outputs[0]: normalized_image})
        results = np.squeeze(results)
        prediction_idx = np.argmax(results)
        prediction = self.labels[prediction_idx])

        return prediction
