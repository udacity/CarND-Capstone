from styx_msgs.msg import TrafficLight

import time
import numpy as np
import os
import sys
import tarfile
import rospy

import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from PIL import Image

category_index = {1: 'red', 2: 'yellow', 3: 'green', 4: 'none'}
result = []
min_prob = 0.50

class TLClassifier(object):
    def __init__(self):
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(rospy.get_param('~model'), 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                # the array based representation of the image will be used later in order to prepare the
                # result image with boxes and labels on it.
                image_np = image #load_image_into_numpy_array(image)
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                # Actual detection.
                #print("doing detection")
                #start_time = time.time()
                (boxes, scores, classes, num) = sess.run(
                [detection_boxes, detection_scores, detection_classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})
                # your code
                elapsed_time = time.time() - start_time
                #print("done")
                #print(elapsed_time)
                # Visualization of the results of a detection.
                max_score_index = np.argmax(scores[0])
                max_score = scores[0][max_score_index]
                if max_score > min_prob:
                    predicted_light = category_index[classes[0][max_score_index]]
                else:
                    predicted_light = category_index[4]
                #print("Predicted Light: {}".format(predicted_light))

                if predicted_light == "red":
                    return TrafficLight.RED
                if predicted_light == "yellow":
                    return TrafficLight.YELLOW
                if predicted_light == "green":
                    return TrafficLight.GREEN
                if predicted_light == "none":
                    return TrafficLight.UNKNOWN
                    
                return TrafficLight.UNKNOWN
