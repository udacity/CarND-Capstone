#!/usr/bin/env python
import cv2
import os
import rospy
import numpy as np
import tensorflow as tf
from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        pwd = os.getcwd()

        self.PATH_TO_CKPT=os.path.join(pwd,'light_classification/frozen_inference_graph.pb')
        #self.PATH_TO_LABELS=os.path.join(pwd,'light_classification/udacity_label_map.pbtxt')
        self.NUM_CLASSES = 4

        #label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.category_index = {1: {'id': 1, 'name': 'Green'},
                               2: {'id': 2, 'name': 'Red'},
                               3: {'id': 3, 'name': 'Yellow'},
                               4: {'id': 4, 'name': 'off'}}

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT,'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        self.sess = tf.Session(graph = self.detection_graph,config=config)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.traffic_light = TrafficLight.UNKNOWN


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_np_expanded = np.expand_dims(image_rgb, axis=0)
        (boxes, scores, classes, num_detections) = self.sess.run(
                    [self.boxes, self.scores, self.classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        for i in range(boxes.shape[0]):
            if scores is None or scores[i]>0.5:
                class_name = self.category_index[classes[i]]['name']
                if class_name == 'Red':
                    self.traffic_light = TrafficLight.RED
                elif class_name == 'Yellow':
                    self.traffic_light = TrafficLight.YELLOW
                elif class_name == 'Green':
                    self.traffic_light = TrafficLight.GREEN
                else:
                    self.traffic_light = TrafficLight.UNKNOWN
        rospy.loginfo('Traffic Light is {}'.format(self.traffic_light))
        return self.traffic_light