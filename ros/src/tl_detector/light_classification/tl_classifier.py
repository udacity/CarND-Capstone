import os
import numpy as np
import tensorflow as tf
import cv2

from styx_msgs.msg import TrafficLight


class TLClassifier(object):

    def __init__(self):
        current_dir = os.path.dirname(__file__)
        
        # We use a SSD detector with a Mobilenet classifier, pretrained on COCO detection task.
        # The frozen graph is compatible with TensorFlow 1.3. It is avilable at the following url:
        # http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz
        # It has been released by the Tensorflow Object Detection API team:
        # https://github.com/tensorflow/models/tree/master/research/object_detection
        PATH_TO_CKPT = os.path.join(current_dir, 'ssd_mobilenet_v1_coco_11_06_2017', 'frozen_inference_graph.pb')

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.tf_session = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # Some of this code is adapted from the TensorFlow Object Detection API tutorial.
        image = cv2.resize(image, (298, 224), interpolation=cv2.INTER_LINEAR)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # Actual detection.
        (boxes, scores, classes, num_detections) = self.tf_session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
        # Extract the indices corresponding to detection of traffic lights (Traffic Light is COCO class number 10).
        traffic_light_indices = np.arange(classes.size)[classes[0] == 10]
        for traffic_light_i in traffic_light_indices[:10]:
            box = boxes[0, traffic_light_i]
            height, width = image.shape[:2]
            box[0] *= height
            box[2] *= height
            box[1] *= width
            box[3] *= width
            box = np.round(box).astype(np.int)
            box[2] = np.minimum(box[2] + 1, height)
            box[3] = np.minimum(box[3] + 1, width)
            y1, x1, y2, x2 = box
            img_crop = image[y1:y2, x1:x2]
            score_r = np.max(np.exp(-np.sum((img_crop - [250, 50, 50]) ** 2, axis=2) / 255))
            score_y = np.max(np.exp(-np.sum((img_crop - [250, 250, 50]) ** 2, axis=2) / 255))
            score_g = np.max(np.exp(-np.sum((img_crop - [60, 250, 80]) ** 2, axis=2) / 255))
            score = np.max([score_r, score_y, score_g])
            if score > 1e-4:
                if score_r > score_y and score_r > score_g:
                    return TrafficLight.RED
                elif score_y > score_g:
                    return TrafficLight.YELLOW
                else:
                    return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
