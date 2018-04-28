import os
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
from PIL import Image
from PIL import ImageDraw
from scipy.stats import norm
import scipy
import scipy.misc
import time
import cv2

SSD_GRAPH_FILE = '/home/student/projects/capstone/ros/frozen_models/ssd_inception_v2_coco_2017_11_17/frozen_inference_graph.pb'
confidence_cutoff = 0.3  # confidence to detect object and edge
padx = 20   # the padding from the boundary 
pady = 20   # the padding from the boundary 
red_green_contrast = 1.8 # if red color is 1.8 times more than green color in the bounding box, treat as red light.
type_traffic_light = 10.0 # seems class 10.0 is for traffic light like thing

class TLClassifier(object):
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

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords


    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph
    
    
    def __init__(self):

        detection_graph = self.load_graph(SSD_GRAPH_FILE)
        # detection_graph = load_graph(RFCN_GRAPH_FILE)
        # detection_graph = load_graph(FASTER_RCNN_GRAPH_FILE)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.detection_graph = detection_graph
    
    def get_classification(self, image):

        res_cv_image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
        
        image_np = np.expand_dims(np.asarray(res_cv_image, dtype=np.uint8), 0)

        with tf.Session(graph=self.detection_graph) as sess:                
            time_before_nn = time.time()
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)


            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

            # The current box coordinates are normalized to a range between 0 and 1.
            # This converts the coordinates actual location on the image.
            width = len(image[1,:])
            height = len(image[:,1])
            box_coords = self.to_image_coords(boxes, height, width)

            # img = Image.fromarray(image, 'RGB')
            # draw = ImageDraw.Draw(img) 
            # for i in range(len(box_coords)):
            #     top, left, bot, right = box_coords[i, ...]
            #     print(len(box_coords), top, left, bot, right)
            #     draw.line([(left, top), (right, bot)], fill=200)

            # scipy.misc.imsave('/home/student/projects/capstone/ros/src/tl_detector/debug_pics/outfile' + 
            #                     str(timeit.timeit()) + '.jpg', img)

        time_after_nn = time.time()

        print("NN process time:" + str(time_after_nn - time_before_nn))
        # compare the red color with green color
        for i in range(len(box_coords)):
            if classes[i] == type_traffic_light: 
                top, left, bot, right = box_coords[i, ...]
                r, g, b = 0, 0, 0

                for x in range(int(left) + padx, int(right) - padx):
                    for y in range(int(top) + pady, int(bot) - pady):
                        p = image[y, x]
                        if(p[0] > 150):
                            r = r + p[0]
                        if(p[1] > 150):
                            g = g + p[1]
                        if(p[2] > 150):
                            b = b + p[2]

                print("r, g, b:")
                print(r, g, b)

                if float(r) > 10 * float(g):
                     return TrafficLight.RED

        return TrafficLight.GREEN
