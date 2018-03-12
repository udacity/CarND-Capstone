from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np

# BEGIN TEST CODE
import matplotlib
# Force matplotlib to not use any Xwindows backend.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor
# END TEST CODE

import cv2
import math
import time
import os

DEBUG_LEVEL = 2  # 0 no Messages, 1 Important Stuff, 2 Everything

# Reference: Object Detection Lab Code

# BEGIN TEST CODE
# Colors (one for each class)
READ_TEST_IMAGE = False
WRITE_BOXES_IMAGE = True
WRITE_DETECTION_IMAGE = True
cmap = ImageColor.colormap
print("Number of colors =", len(cmap))
COLOR_LIST = sorted([c for c in cmap.keys()])
# END TEST CODE

def mobilenet_conv_block(x, kernel_size, output_channels):
    """
    Depthwise Conv -> Batch Norm -> ReLU -> Pointwise Conv -> Batch Norm -> ReLU
    """
    # assumes BHWC format
    input_channel_dim = x.get_shape().as_list()[-1]
    W = tf.Variable(tf.truncated_normal((kernel_size, kernel_size, input_channel_dim, 1)))

    # depthwise conv
    x = tf.nn.depthwise_conv2d(x, W, (1, 2, 2, 1), padding='SAME')
    x = tf.layers.batch_normalization(x)
    x = tf.nn.relu(x)

    # pointwise conv
    x = tf.layers.conv2d(x, output_channels, (1, 1), padding='SAME')
    x = tf.layers.batch_normalization(x)

    return tf.nn.relu(x)

def filter_boxes(min_score, boxes, scores, classes):
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

def to_image_coords(boxes, height, width):
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

def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

# BEGIN TEST CODE
def draw_boxes(image, boxes, classes, run_time, light_debug_index):
    """Draw bounding boxes on the image"""
    if not READ_TEST_IMAGE:
        image = Image.fromarray(image)
    draw = ImageDraw.Draw(image)
    for i in range(len(boxes)):
        bot, left, top, right = boxes[i, ...]
        class_id = int(classes[i])
        color = COLOR_LIST[class_id]
        draw.line([(left, top), (left, bot), (right, bot), (right, top), (left, top)], width=4, fill=color)

    plt.figure(figsize=(12, 8))
    plt.imshow(image)
    plt.savefig(run_time + "/boxes_image" + str(light_debug_index) + ".jpg")
# END TEST CODE

class TLClassifier(object):
    def __init__(self):

        # BEGIN TEST CODE
        plt.style.use('ggplot')

        # constants but you can change them so I guess they're not so constant :)
        INPUT_CHANNELS = 32
        OUTPUT_CHANNELS = 512
        KERNEL_SIZE = 3
        IMG_HEIGHT = 256
        IMG_WIDTH = 256

        with tf.Session(graph=tf.Graph()) as sess:
            # input
            x = tf.constant(np.random.randn(1, IMG_HEIGHT, IMG_WIDTH, INPUT_CHANNELS), dtype=tf.float32)

            with tf.variable_scope('mobile'):
                mobilenet_conv = mobilenet_conv_block(x, KERNEL_SIZE, OUTPUT_CHANNELS)

            mobile_params = [
                (v.name, np.prod(v.get_shape().as_list()))
                for v in tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'mobile')
            ]
        # END TEST CODE

        # Detection Initialization

        SSD_GRAPH_FILE = './ssd_frozen_inference_graph.pb'
        RFCN_GRAPH_FILE = './rfcn_frozen_inference_graph.pb'
        FASTER_RCNN_GRAPH_FILE = './faster_rcnn_frozen_inference_graph.pb'
        self.detection_graph = load_graph(SSD_GRAPH_FILE)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.light_debug_index = 0
        self.run_time = time.strftime("%H:%M:%S")
        if DEBUG_LEVEL >= 2:
            if not os.path.exists("./" + self.run_time):
                os.makedirs("./" + self.run_time)

    def traffic_light_detection(self, image):
        """Detect a traffic light in the image and return it in a new cropped image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            imaged cropped to a box around the traffic light
        """
        light_image = None

        # BEGIN TEST CODE
        if READ_TEST_IMAGE:
            image = Image.open('assets/sample1.jpg')
        # END TEST CODE

        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        with tf.Session(graph=self.detection_graph) as sess:
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes],
            feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)

            confidence_cutoff = 0.6
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = filter_boxes(confidence_cutoff, boxes, scores, classes)

            # The current box coordinates are normalized to a range between 0 and 1.
            # This converts the coordinates actual location on the image.

            # BEGIN TEST CODE
            if READ_TEST_IMAGE:
                width, height = image.size
            else:
            # END TEST CODE
                height, width, channels = image.shape

            box_coords = to_image_coords(boxes, height, width)

            # BEGIN TEST CODE
            # Each class with be represented by a differently colored box
            draw_boxes(image, box_coords, classes, self.run_time, self.light_debug_index)
            # END TEST CODE

            for i in range(len(boxes)):
                bot, left, top, right = box_coords[i, ...]
                class_id = int(classes[i])
                if class_id == 10:
                    light_image = image[int(bot):int(top), int(left):int(right)]
                    if DEBUG_LEVEL >= 2:
                        print("TL Classifier image", width, height, "light box", self.run_time, int(bot), int(top), int(left), int(right))
                        if not READ_TEST_IMAGE and WRITE_BOXES_IMAGE:
                            cv2.imwrite(self.run_time + "/camera_image" + str(self.light_debug_index) + ".jpg", image)
                        if not READ_TEST_IMAGE and WRITE_BOXES_IMAGE:
                            cv2.imwrite(self.run_time + "/light_image" + str(self.light_debug_index) + ".jpg", light_image)
                        self.light_debug_index += 1
        return light_image

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction

        image = self.traffic_light_detection(image)

        return TrafficLight.UNKNOWN
