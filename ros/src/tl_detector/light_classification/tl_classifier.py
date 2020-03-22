from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np
import os
GRAPH_FILE='/frozen_inference_graph.pb'


class TLClassifier(object):
    def __init__(self):
        cwd=os.path.dirname(os.path.realpath(__file__))
        print cwd
        self.graph_path = cwd + GRAPH_FILE
        #load tensorflow graph file
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.graph_path,'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        image_np=np.expand_dims(np.asarray(image_rgb, dtype=np.uint8), 0)
        with tf.Session(graph=self.detection_graph) as sess:
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes],
                                                feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)

            confidence_cutoff = 0.8
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
            count_red = 0
            count_yellow = 0
            count_green = 0
            for n in classes:
                if int(n) == 1:
                    count_red+=1
                elif int(n) == 2:
                    count_green+=1
                else:
                    count_yellow+=1
                if count_red>0:
                    return TrafficLight.RED
                elif count_yellow>0:
                    return TrafficLight.YELLOW
                else:
                    return TrafficLight.GREEN
        return TrafficLight.UNKNOWN

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes
