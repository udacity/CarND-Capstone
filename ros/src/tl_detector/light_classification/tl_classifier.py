from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import numpy as np
import cv2

def select_boxes(boxes, classes, scores, score_threshold=0, target_class=10):
    """
    :param boxes:
    :param classes:
    :param scores:
    :param target_class: default traffic light id in COCO dataset is 10
    :return:
    """
    sq_scores = np.squeeze(scores)
    sq_classes = np.squeeze(classes)
    sq_boxes = np.squeeze(boxes)

    sel_id = np.logical_and(sq_classes == target_class, sq_scores > score_threshold)
    return sq_boxes[sel_id]


def crop_roi_image(image_np, sel_box):
    im_height, im_width, _ = image_np.shape
    (left, right, top, bottom) = (sel_box[1] * im_width, sel_box[3] * im_width,
                                  sel_box[0] * im_height, sel_box[2] * im_height)
    cropped_image = image_np[int(top):int(bottom), int(left):int(right), :]
    return cropped_image


class TLClassifier(object):
    def __init__(self):
        self.load_graph()
        self.extract_graph_components()
        self.sess = tf.Session(graph=self.detection_graph)

        pass

    def extract_graph_components(self):
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def detect_multi_object(self, image_np, score_threshold):
        """
        Return detection boxes in a image

        :param image_np:
        :param score_threshold:
        :return:
        """

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.

        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        sel_boxes = select_boxes(boxes=boxes, classes=classes, scores=scores,
                                 score_threshold=score_threshold, target_class=10)

        return sel_boxes

    def load_graph(self):

        # To read more about the model zoo see:
        # https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md
        PATH_TO_CKPT = "./light_classification/models/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb"

        cwd = os.getcwd()
        print(cwd)

        detection_graph = tf.Graph()
        with detection_graph.as_default():
            # Line below is different from tensorflow 2 version used in Jupyter
            # https://stackoverflow.com/questions/57614436/od-graph-def-tf-graphdef-attributeerror-module-tensorflow-has-no-attribut
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.detection_graph = detection_graph

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        #Parameters
        MIN_AREA_FOR_STOP = 3
        MIN_COMPACTNESS_FOR_BULB = 0.04

        mask, area, perimeter = extract_red_area(image)

        if area > MIN_AREA_FOR_STOP and area/(perimeter*perimeter)> MIN_COMPACTNESS_FOR_BULB:
            state = TrafficLight.RED  # state = 'stop' >> id = 0
            #print(area/(perimeter*perimeter))
        else:
            state = TrafficLight.GREEN    # state = 'go' >> id = 2

        return state

        #for img in images:
        #    mask, area, perimeter = extract_red_area(img)
        #    if area > MIN_AREA_FOR_STOP and area/(perimeter*perimeter)> MIN_COMPACTNESS_FOR_BULB:
        #        state = 'stop'
        #        #print(area/(perimeter*perimeter))
        #    else:
        #        state = 'go'


    def extract_red_area(self, img):

        LOWER_RED = np.array([90-30,30,40])
        UPPER_RED = np.array([90+30,255,255])

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_inv = cv2.bitwise_not(img)
        hsv=cv2.cvtColor(img_inv, cv2.COLOR_RGB2HSV);
        mask = cv2.inRange(hsv, LOWER_RED, UPPER_RED)
        im2, contours,hierarchy = cv2.findContours(mask, 1, 2)
        largest_area = 0
        perimeter = 0
        if(contours):
            for contour in contours:
                area = cv2.contourArea(contour)
                if area>largest_area:
                    largest_area=area
                    perimeter = cv2.arcLength(contour, True)

        return mask, largest_area, perimeter

    def detect_traffic_light(self, image):
        boxes = self.detect_multi_object(image, score_threshold=0.2)
        if len(boxes) == 0:
            return np.array([])

        cropped_image = crop_roi_image(image, boxes[0])
        return cropped_image
