from styx_msgs.msg import TrafficLight

import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from object_detect_api import label_map_util
from object_detect_api import visualization_utils as vis_util


from io import StringIO

import time

class TLClassifier(object):
    def __init__(self, run_mode):

        # Default value returned by the classifier
        self.detected_light_state = TrafficLight.UNKNOWN
        # Source is simulation by default, set to false for loading the classifier for real data
        self.enabled = True

        #Detection result image
        self.detection_image = None

        cwd = os.path.dirname(os.path.realpath(__file__))

        if run_mode == "simulator":
            graph_source_path = cwd + '/networks/faster_rcnn_resnet50_low_fine_tuned_model_4class_v1_sim/frozen_inference_graph.pb'
        elif run_mode == "site":
            graph_source_path = cwd + '/networks/faster_rcnn_resnet50_low_fine_tuned_model_4class_v1_real_train2/frozen_inference_graph.pb'

        else:
            raise ValueError("The class can be initialized with simulator or site mode only ...")

        labels_source_path = cwd + '/networks/label_map_4class.txt'
        num_classes = 4

        label_map = label_map_util.load_labelmap(labels_source_path)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)


        self.inference_graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with self.inference_graph.as_default():
            inference_graph_definition = tf.GraphDef()

            with tf.gfile.GFile(graph_source_path, 'rb') as fid:
                serialized_graph = fid.read()
                inference_graph_definition.ParseFromString(serialized_graph)
                tf.import_graph_def(inference_graph_definition, name='')

            self.sess = tf.Session(graph=self.inference_graph, config=config)

        # Input layer
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Inference graph loaded ...")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.enabled is True:
            image_np_expanded = np.expand_dims(image, axis=0)

            #time0 = time.time()

            # Actual detection.
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores,
                     self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})


            #time1 = time.time()

            #print("Time in milliseconds", (time1 - time0) * 1000)

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)


            # DISTANCE TO TRAFFIC LIGHT and passing to TrafficLight thing
            # Should be done as part of visual to avoid duplicate computation

            min_score_thresh = .50
            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > min_score_thresh:

                    class_name = self.category_index[classes[i]]['name']
                    print('{}'.format(class_name))

                    self.detected_light_state = TrafficLight.UNKNOWN

                    if class_name == 'Red':
                        self.detected_light_state = TrafficLight.RED
                    elif class_name == 'Green':
                        self.detected_light_state = TrafficLight.GREEN
                    elif class_name == 'Yellow':
                        self.detected_light_state = TrafficLight.YELLOW


            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image, boxes, classes, scores,
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8)
            self.detection_image = image

        return self.detected_light_state
