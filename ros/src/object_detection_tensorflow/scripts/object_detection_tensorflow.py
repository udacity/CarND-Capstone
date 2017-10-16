#!/usr/bin/env python
from __future__ import print_function
# import roslib
# roslib.load_manifest('object_detection_tensorflow')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from glob import glob
from tqdm import tqdm

from utils import label_map_util

from utils import visualization_utils as vis_util


cwd = os.getcwd()
print("CWD:",cwd)
PROJECT_PATH = cwd
# PROJECT_PATH = '/home/karol/projects/udacity/CARND/Term3/CarND-Capstone/ros/src/object_detection_tensorflow/'

MODEL_NAME = PROJECT_PATH + '/graph/run15-5957'
# MODEL_NAME = PROJECT_PATH + '/graph/run16-25761'
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
PATH_TO_LABELS = PROJECT_PATH + "/data/traffic_lights.pbtxt"
NUM_CLASSES = 3

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')



        class image_converter:
          written_images_counter=0
          def __init__(self):
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)

          def callback(self,data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                image_np = cv2. cvtColor(cv_image,cv2.COLOR_BGR2RGB)

                results = []

                image = image_np
                image_np_expanded = np.expand_dims(image, axis=0)
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})

                vis_util.visualize_boxes_and_labels_on_image_array(
                  image,
                  np.squeeze(boxes),
                  np.squeeze(classes).astype(np.int32),
                  np.squeeze(scores),
                  category_index,
                  min_score_thresh=0.05,
                  use_normalized_coordinates=True,
                  line_thickness=8)


                result = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
                cv2.imwrite('/tmp/image%08d.jpg'%self.written_images_counter,result)
                self.written_images_counter = self.written_images_counter + 1
                cv2.imshow('result',result)
                cv2.waitKey(10)

            except CvBridgeError as e:
              print(e)
        def main(args):
          ic = image_converter()
          rospy.init_node('object_detection_tensorflow', anonymous=True)
          try:
            rospy.spin()
          except KeyboardInterrupt:
            print("Shutting down")
          cv2.destroyAllWindows()

        if __name__ == '__main__':
            main(sys.argv)
