from styx_msgs.msg import TrafficLight

import tensorflow as tf
from tensorflow.python.platform import gfile
from tensorflow.python.util import compat
from tensorflow.core.protobuf import saved_model_pb2
import os
import cv2
import numpy as np
import rospy

def load_graph(graph_file, config, verbose = False):
    with tf.Session(graph=tf.Graph(), config=config) as sess:
        assert tf.get_default_session() is sess
        gd = tf.GraphDef()
        with tf.gfile.Open(graph_file, 'rb') as f:
            data = f.read()
            gd.ParseFromString(data)
        tf.import_graph_def(gd, name='')
        graph = tf.get_default_graph()
        if verbose:
            print ('Graph v' + str(graph.version) + ', nodes: '+ ', '.join([n.name for n in graph.as_graph_def().node]))
        return graph

def extractBox(boxes, scores, classes, confidence, im_width, im_height):
    # Prepare stuff
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    scores = np.squeeze(scores)

    # Get bounding box with highest confidence
    maxConf = 0
    number = -1
    for i in range(boxes.shape[0]):
        if scores[i] > confidence and classes[i] == 10:
            if scores[i] > maxConf:
                maxConf = scores[i]
                number = i

    if number != -1:
        # Create a tuple for earch box
        box = tuple(boxes[number].tolist())

        # Extract box corners
        ymin, xmin, ymax, xmax = box
        (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                            ymin * im_height, ymax * im_height)

        # Expand them a little bit
        left = left - 5
        if left < 0:
            left = 0
        top = top - 10
        if top < 0:
            top = 0
        bottom = bottom + 10
        if bottom > im_height:
            bottom = im_height
        right = right + 5
        if right > im_width:
            right = im_width
        box = int(left), int(right), int(top), int(bottom)
        return box

    else:
        return None

class TLClassifier(object):
    def __init__(self, simulator):
        #TODO load classifier
        self.simulator = simulator

        # Activate optimizations for TF
        self.config = tf.ConfigProto()
        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level

        # self.graph_classification = load_graph('models/model_classification.pb', self.config)

        # if simulator:
            # self.graph_detection = load_graph('models/model_detection_simulator.pb', self.config)
        # else:
            # self.graph_detection = load_graph('models/model_detection_site.pb', self.config)
        rospy.loginfo("Models loaded!")

        pass




    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN

if __name__ == "__main__":
    classifier = TLClassifier(simulator = True)
