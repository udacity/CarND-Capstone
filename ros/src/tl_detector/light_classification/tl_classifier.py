import os

import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self, model_name):
        self.current_light = TrafficLight.UNKNOWN

        cwd = os.path.dirname(os.path.realpath(__file__))

        model_path = os.path.join(cwd, "model/{}".format(model_name))
        rospy.logwarn("model_path={}".format(model_path))

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.color_index = {1: {'id': 1, 'name': 'Green'}, 2: {'id': 2, 'name': 'Red'},
                            3: {'id': 3, 'name': 'Yellow'}, 4: {'id': 4, 'name': 'off'}}

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(graph=self.detection_graph, config=config)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.size = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (width, height, _) = image.shape
        image_np = np.expand_dims(image, axis=0)

        with self.detection_graph.as_default():
            (boxes, scores, classes, size) = self.sess.run(
                [self.boxes, self.scores,
                 self.classes, self.size],
                feed_dict={self.image_tensor: image_np})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = .5

        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                class_name = self.color_index[classes[i]]['name']
                if class_name == 'Red':
                    return TrafficLight.RED
        return TrafficLight.GREEN
