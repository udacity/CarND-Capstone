import os

import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        self.current_light = TrafficLight.UNKNOWN
        #model = 'frozen_inference_graph.pb'
        model = 'frozen_inference_graph_sim_tf_v1.4.pb'
        model_path = 'light_classification/ssd_mobilenet_v1_coco_11_06_2017/' + model
        print(os.getcwd())
        self.graph = tf.Graph()
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
               serialized_graph = fid.read()
               od_graph_def.ParseFromString(serialized_graph)
               tf.import_graph_def(od_graph_def, name='')
            
        self.category_index = {1: {'id': 1, 'name': 'Green'}, 2: {'id': 2, 'name': 'Red'},
                               3: {'id': 3, 'name': 'Yellow'}, 4: {'id': 4, 'name': 'off'}}

        # create tensorflow session for detection
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # end
        self.sess = tf.Session(graph=self.graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (im_width, im_height, _) = image_rgb.shape
        image_np = np.expand_dims(image_rgb, axis=0)

        # Actual detection.
        with self.graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = .5
        count = 0
        count1 = 0
        # print(scores)

        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                count1 += 1
                class_name = self.category_index[classes[i]]['name']

                # Traffic light thing
                if class_name == 'Red':
                    count += 1

        # print(count)
        if count < count1 - count:
            self.current_light = TrafficLight.GREEN
        else:
            self.current_light = TrafficLight.RED

        return self.current_light