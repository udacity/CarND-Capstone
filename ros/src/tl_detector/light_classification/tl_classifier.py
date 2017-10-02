from styx_msgs.msg import TrafficLight
import rospy

import os
import cv2
import sys
import time
import tensorflow as tf
import numpy as np
from matplotlib import pyplot as plt
import argparse

from collections import defaultdict
from io import StringIO
from PIL import Image
import matplotlib.image as mping

# there is a tensorflow dependency problem. 
# the following solution is carrying from @Anthony  
# https://github.com/nhiddink/SDCND_Capstone_TEC
from utilities import label_map_util
from utilities import visualization_utils as vis_util


class TLClassifier(object):
    def __init__(self):
	self.state = TrafficLight.UNKNOWN

        #TODO load classifier
	#sys.path.append('/home/studeng/Carnd-Capstone/ros')
	#cwd = os.path.dirname(os.path.realpath(__file__))
	CWD_PATH = os.getcwd()
	MODEL_NAME = 'model'
	#CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
	CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')
	#PATH_TO_LABELS = MODEL_NAME +'/label_map.pbtxt'
	PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, 'label_map.pbtxt')
	NUM_CLASSES = 14
        	
	self.detection_graph = tf.Graph()

	with self.detection_graph.as_default():
  	    od_graph_def = tf.GraphDef()
  	    with tf.gfile.GFile(CKPT, 'rb') as fid:
    	    	serialized_graph = fid.read()
            	od_graph_def.ParseFromString(serialized_graph)
   	    	tf.import_graph_def(od_graph_def, name='')

	label_map      = label_map_util.load_labelmap(PATH_TO_LABELS)
	categories     = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
	self.category_index = label_map_util.create_category_index(categories)

    def load_image_into_numpy_array(self, image):
        #rospy.loginfo(image.shape)
  	(im_width, im_height) = image.size
	#im_width = image.shape[0]
        #im_height = image.shape[1]
  	return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)
	#im_arr = np.fromstring(image.tobytes(), dtype=np.uint8)
	#return im_arr.reshape((image.size[1], image.size[0], 3))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
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
		image = Image.fromarray(image)
            	image_np = self.load_image_into_numpy_array(image)
            	# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            	image_np_expanded = np.expand_dims(image_np, axis=0)
		#image_np_expanded = np.expand_dims(image, axis=0)

            	time0 = time.time()

            	# Actual detection.
            	(boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
              						feed_dict={image_tensor: image_np_expanded})

            	time1 = time.time()

            	#print("Time in milliseconds", (time1 - time0) * 1000) 

		boxes = np.squeeze(boxes)
            	scores = np.squeeze(scores)
            	classes = np.squeeze(classes).astype(np.int32)

		min_score_thresh = .50
            	for i in range(boxes.shape[0]):
                    if scores is None or scores[i] > min_score_thresh:    
                        class_name = self.category_index[classes[i]]['name']
                        class_id = self.category_index[classes[i]]['id']  # if needed

                        print('{}'.format(class_name))

                        if class_name == 'Red':
                            self.state = TrafficLight.RED
                        elif class_name == 'Green':
                            self.state = TrafficLight.GREEN
                        elif class_name == 'Yellow':
                            self.state = TrafficLight.YELLOW

                # Visualization of the results of a detection.
            	vis_util.visualize_boxes_and_labels_on_image_array(image_np,
              							   np.squeeze(boxes),
              							   np.squeeze(classes).astype(np.int32),
              							   np.squeeze(scores),
              				                           self.category_index,
                                                                   use_normalized_coordinates=True,
               							   line_thickness=3)

        return self.state
