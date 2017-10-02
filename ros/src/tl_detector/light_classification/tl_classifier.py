from styx_msgs.msg import TrafficLight
import numpy as np
import os
import sys
import tensorflow as tf
import label_map_util
import visualization_utils as vis_util

class TLClassifier(object):
	def __init__(self):

		# Properties
		self.is_loaded = False
		self.current_light = TrafficLight.UNKNOWN
		self.detection_graph = tf.Graph()

		# Prepare category list needed for preview
		label_map = label_map_util.load_labelmap(os.path.dirname(os.path.realpath(__file__))+"/label_map.pbtxt")
		categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=14, use_display_name=True)
		self.category_index = label_map_util.create_category_index(categories)

		# Load model graph
		model_path = os.path.dirname(os.path.realpath(__file__))+"/../../../../../model.pb"
		
		config = tf.ConfigProto()
		config.gpu_options.allow_growth = True # https://github.com/tensorflow/tensorflow/issues/6698

		with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()

			with tf.gfile.GFile(model_path, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')

			self.sess = tf.Session(graph=self.detection_graph, config=config)

		self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
		self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
		self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
		self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
		self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

		self.is_loaded = True
		print("TF Classifier loaded")

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""

		# Classify
		image_np_expanded = np.expand_dims(image, axis=0)
		with self.detection_graph.as_default():
			(boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: image_np_expanded})
		boxes = np.squeeze(boxes)
		scores = np.squeeze(scores)
		classes = np.squeeze(classes).astype(np.int32)

		# Preview classified bouding boxes
		vis_util.visualize_boxes_and_labels_on_image_array(image, boxes, classes, scores, self.category_index, use_normalized_coordinates=True)

		# Analyse classification and decide about final traffic light value
		min_score_thresh = .50
		classified_light = TrafficLight.UNKNOWN
		for i in range(boxes.shape[0]):
			if scores is None or scores[i] > min_score_thresh:
				
				class_index = classes[i]
				class_name = self.category_index[class_index]["name"]
				print('Classification {} {}'.format(class_index, class_name))

				# Tranform model classes into TrafficLight enums
				if class_index == 2 or class_index == 5 or class_index == 6 or class_index == 9 or class_index == 13 or class_index == 14:
					classified_light = TrafficLight.RED
				elif class_index == 1 or class_index == 3 or class_index == 4 or class_index == 10 or class_index == 11 or class_index == 12:
					classified_light = TrafficLight.GREEN
				elif class_index == 7:
					classified_light = TrafficLight.YELLOW
		
		self.current_light = classified_light

		return self.current_light, image