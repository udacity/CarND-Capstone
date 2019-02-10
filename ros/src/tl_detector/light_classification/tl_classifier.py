import numpy as np
import os
import tensorflow as tf

from styx_msgs.msg import TrafficLight

FROZEN_SIM_INFERENCE_GRAPH = os.getcwd() + "/sim_traffic_light_graph.pb"
FROZEN_SITE_INFERENCE_GRAPH = os.getcwd() + "/site_traffic_light_graph.pb"
SCORE_THRESHOLD = 0.5
MAX_BOXES = 3

class TLClassifier(object, is_site=False):
    def __init__(self):
        if (is_site):
            graph_loc = FROZEN_SITE_INFERENCE_GRAPH
        else:
            graph_loc = FROZEN_SIM_INFERENCE_GRAPH

        #load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_loc, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            # end with

            self.sess = tf.Session(graph=self.detection_graph)

            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # end with

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #implement light color prediction
        state = TrafficLight.UNKNOWN
        with self.detection_graph.as_default():
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_expanded = np.expand_dims(image, axis=0)
            # Actual detection.
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})

            boxes = np.squeeze(boxes)
            classes = np.squeeze(classes).astype(np.int32)
            scores = np.squeeze(scores)

            max_score = 0
            for i in range(min(MAX_BOXES, boxes.shape[0])):
                if (scores[i] > SCORE_THRESHOLD) and (scores[i] > max_score):
                    if (classes[i] == 1):
                        state = TrafficLight.GREEN
                    elif (classes[i] == 2):
                        state = TrafficLight.RED
                    elif (classes[i] == 3):
                        state = TrafficLight.YELLOW
                    elif:
                        state = TrafficLight.UNKNOWN
                    max_score = scores[i]
            # end for
        # end with

        return state