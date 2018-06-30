from styx_msgs.msg import TrafficLight
import tensorflow as tf
import rospy
import cv2
import numpy as np


# Utility
def imageLoadUtil(image):
    return np.asarray(image, dtype="uint8")

class TLClassifier(object):
    def __init__(self):
		# downloaded from Tensorflow website
		# http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2017_11_17.tar.gz
		
        self.number_of_images = 0

        PATH_TO_CKPT = 'perception/frozen_inference_graph.pb'

        self.model = None
        self.width = 0
        self.height = 0
        self.channels = 3

		# Load Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()
			with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')

			self.sess = tf.Session(graph=self.detection_graph)
			self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


    def init_classifier(self, model, width, height, channels=3):
        self.width = width
        self.height = height
        self.model = model
        self.channels = channels
        self.graph = tf.get_default_graph()

    def tl_light_classifier(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
		# Resize and Normalize
        norImage = cv2.resize(image, (32,64)) / 255.;
		
        with self.graph.as_default():
            predictions = self.model.predict(norImage.reshape((1, 64, 32, self.channels)))
            color =  predictions[0].tolist().index(np.max(predictions[0]))
            tl = TrafficLight()
            tl.state = color
            return tl.state

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

        for idx, classID in enumerate(classes):
			# Traffic light class ID 
            if classID == 10:
                break

        temporary = (0, 0, 0, 0)
		
		# Uncertain prediction
        if scores[idx] < 0.1:
            return TrafficLight.UNKNOWN, temporary

        nbox = bbox[idx]

        boundary = np.array([nbox[0]*image.shape[0], nbox[1]*image.shape[1], nbox[2]*image.shape[0], nbox[3]*image.shape[1]]).astype(int)
        imageTl = image[boundary[0]:boundary[2], boundary[1]:boundary[3]]
        result = self.tl_light_classifier(imageTl)
		
        return  result, boundary
