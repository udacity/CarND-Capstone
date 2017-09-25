from styx_msgs.msg import TrafficLight

import numpy as np
import os
import sys
import tensorflow as tf

from collections import defaultdict
from io import StringIO

from utilities import label_map_util
from utilities import visualization_utils as vis_util

import cv2

#Testing
#from matplotlib import pyplot as plt


class TLClassifier(object):
    def __init__(self):
        self.current_light = TrafficLight.UNKNOWN
        cwd = os.path.dirname(os.path.realpath(__file__))

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        base_path = os.path.dirname(os.path.abspath(__file__))
        MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
        PATH_TO_CKPT = os.path.join(base_path, MODEL_NAME, 'frozen_inference_graph.pb')

        # Load label map
        PATH_TO_LABELS = os.path.join(base_path, 'data', 'mscoco_label_map.pbtxt')
        NUM_CLASSES = 90
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        # Build network
        self.detection_graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True  # https://github.com/tensorflow/tensorflow/issues/6698

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Classifier initialisation complete!")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image_np_expanded = np.expand_dims(image, axis=0)

        # Perform network inference
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        # Loop through the detections which are TRAFFIC LIGHTS and get the bounding box for
        # the highest score. If above a THRESHOLD (0.1) then crop the image with this
        # bounding box.
        # With the cropped index perform a basic CV colour histogram to determine the maximum
        # colour of green, orange/yellow, red.
        best_score = 0.
        best_score_index = 0

        for i in range(0, classes.size - 1):
            if classes[i] == 10.:
                #score_as_list = tuple(scores.tolist())
                #score = score_as_list[i]
                score = scores[i]
                if score > best_score:
                    best_score = score
                    best_score_index = i

        print("Best Score:", best_score, best_score_index)

        if best_score > .1:
            #box_list = tuple(boxes[0].tolist())
            #box = box_list[best_score_index]
            box = boxes[best_score_index]

            #im_width, im_height = image.size
            im_height, im_width, im_depth = image.shape
            ymin, xmin, ymax, xmax = box
            (left, right, top, bottom) = (xmin * im_width, xmax * im_width, ymin * im_height, ymax * im_height)

            # Get rid of borders by shrinking image a little
            left_int = np.uint16(left) + 10
            right_int = np.uint16(right) - 5
            top_int = np.uint16(top) + 5
            bottom_int = np.uint16(bottom) - 5

            #tf_image_cropped = image_np_expanded[top_int:bottom_int, left_int:right_int, :]
            tf_image_cropped = image[top_int:bottom_int, left_int:right_int, :]
            gray = cv2.cvtColor(tf_image_cropped.astype('uint8'), cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (11, 11), 0)  # 11 is the radius (must be odd no.)
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)  # maxLoc is (x, y)

            # Is this a vertical or horizontal traffic light
            gray_height, gray_width = gray.shape
            if gray_height >= gray_width:
                VERTICAL_LIGHT = True
            else:
                VERTICAL_LIGHT = False

            # Classify the light based on position of brightest area
            if VERTICAL_LIGHT == True:
                (maxLoc_width, maxLoc_height) = maxLoc

                if float(maxLoc_height) / float(gray_height) > 0.7:
                    self.current_light = TrafficLight.GREEN
                elif float(maxLoc_height) / float(gray_height) > 0.4:
                    self.current_light = TrafficLight.YELLOW
                else:
                    self.current_light = TrafficLight.RED
            else:
                self.current_light = TrafficLight.UNKNOWN

            # show bright spot
            #bright_spot_image = tf_image_cropped.copy()
            #cv2.circle(bright_spot_image, maxLoc, 10, (255, 0, 0), 2)  # 10 is circle radius
            #plt.figure(figsize=(12, 8))
            #plt.imshow(tf_image_cropped)
            #plt.show()
        else:
            self.current_light = TrafficLight.UNKNOWN

        return self.current_light
