from styx_msgs.msg import TrafficLight

import tensorflow as tf
from tensorflow.python.platform import gfile
from tensorflow.python.util import compat
from tensorflow.core.protobuf import saved_model_pb2
import os
import cv2
import numpy as np
import rospy
import rospkg

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

        rp = rospkg.RosPack()
        model_dir = os.path.join(rp.get_path('tl_detector'), 'light_classification/models')
        rospy.loginfo(model_dir)

        self.graph_classification = load_graph(model_dir + '/model_classification.pb', self.config)
        if simulator:
            self.graph_detection = load_graph(model_dir + '/model_detection_simulator.pb', self.config)
        else:
            self.graph_detection = load_graph(model_dir + '/model_detection_site.pb', self.config)
        rospy.loginfo("Models loaded!")

        self.session_classification = tf.Session(graph=self.graph_classification, config=self.config)
        self.session_detection = tf.Session(graph=self.graph_detection, config=self.config)

        self.image_tensor = self.graph_detection.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph_detection.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.graph_detection.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.graph_detection.get_tensor_by_name('detection_classes:0')

        self.in_graph = self.graph_classification.get_tensor_by_name('input_1_1:0')
        self.out_graph = self.graph_classification.get_tensor_by_name('output_0:0')

        self.labels = {0: TrafficLight.RED, 1: TrafficLight.GREEN, 2: TrafficLight.YELLOW}


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        box = self.detection(image)
        if box == None:
            return TrafficLight.UNKNOWN

        left, right, top, bottom = box
        img_crop = image[top:bottom, left:right]
        traffic_light = cv2.resize(img_crop, (32, 32))
        return self.classification(traffic_light)

    def detection(self, image):
        im_height, im_width, _ = image.shape
        image_expanded = np.expand_dims(image, axis=0)

        with self.session_detection.as_default(), self.graph_detection.as_default():
            boxes, scores, classes = self.session_detection.run([self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_expanded})
            return extractBox(boxes, scores, classes, 0.1, im_width, im_height)

    def classification(self, image):
        with self.session_classification.as_default(), self.graph_classification.as_default():
            sfmax = list(self.session_classification.run(tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [image]}))))
            sf_ind = sfmax.index(max(sfmax))
            return self.labels[sf_ind]

if __name__ == "__main__":
    classifier = TLClassifier(simulator = True)
