from styx_msgs.msg import TrafficLight
import tensorflow as tf
import rospy
import cv2
import numpy as np

from utilities import label_map_util

# Utility
def imageLoadUtil(image):
    return np.asarray(image, dtype="uint8")

class TLClassifier(object):
    def __init__(self, graph, labels):
        num_classes = 14

        label_map = label_map_util.load_labelmap(labels)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

		# Load Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()
			with tf.gfile.GFile(graph, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')

			self.sess = tf.Session(graph=self.detection_graph)
			self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        npImage = imageLoadUtil(image)
        exnpImage = np.expand_dims(npImage, axis=0)

		# Reference https://github.com/tensorflow/models/issues/1773
        with self.detection_graph.as_default():
            (bbox, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: exnpImage})

        bbox = np.squeeze(bbox)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        min_score_theshold = 0.7

        for idx in range(bbox.shape[0]):
            if scores is None or scores[idx] < min_score_theshold:
                class_name = self.category_index[classes[idx]]['name']

                if class_name == 'Red':
                    result = TrafficLight.RED
                elif class_name == 'Yellow':
                    result = TrafficLight.YELLOW
                elif class_name == 'Green':
                    result = TrafficLight.GREEN

        return  result
