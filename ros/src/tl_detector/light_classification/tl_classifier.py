from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import tensorflow as tf
import rospy
import traceback
import json
import time
import os
import re

dirname = os.path.dirname(__file__)

root_path = re.findall('^/home/.*Capstone/', dirname)[0]

GRAPH_FILE = os.path.join(root_path, 'data/graphs/frozen_inference_graph.pb')


BOXES_SCORE_MIN = 0.5  # Minimum passing score for detection

'''
def log(arg):
    #writes log information to debug file
    with open('logfile.txt','a') as f:
        f.write(arg+'\n')
'''


def load_graph(graph_file):
    # Loads a frozen inference graph
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

# ----------FUNCTIONS FOR CLASSIFICATION----------


def draw_boxes(img, bboxes, color=(0, 0, 255), thick=3):
    # Draws bounding boxes
    # Make a copy of the image
    imcopy = np.copy(img)
    # Iterate through the bounding boxes
    for bbox in bboxes:
        # Draw a rectangle given bbox coordinates
        cv2.rectangle(imcopy, (bbox[1], bbox[0]), (bbox[3], bbox[2]), color, thick)
    # Return the image copy with boxes drawn
    return imcopy


def TLDetection(image, sess):

    image_np = np.expand_dims(np.asarray(image, dtype = np.uint8), 0)
    boxes, scores, classes = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict = {image_tensor: image_np})

    boxes = np.squeeze(boxes)
    scores = np.squeeze(scores)
    classes = np.squeeze(classes)

    return boxes, scores, classes


def TLBoxes(prob, boxes, scores, classes):
    # filter boxes under minimum probability 'prob'
    # COCO class index for TrafficLight is '10'
    n = len(boxes)
    idxs = []
    # target = {1, 2, 3}
    for i in range(n):
        if scores[i] >= prob:
            # if scores[i] >= prob and classes[i] in target:
            idxs.append(i)

    filtered_boxes = boxes[idxs, ...]
    filtered_scores = scores[idxs, ...]
    filtered_classes = classes[idxs, ...]
    # print(filtered_classes)
    # print()
    return filtered_boxes, filtered_scores, filtered_classes


def TLResizeBoxes(boxes, image_height, image_width):
    # Resize boxes from original values (0:1) to image size
    box_coords = np.zeros_like(boxes)
    box_coords[:, 0] = boxes[:, 0] * image_height
    box_coords[:, 1] = boxes[:, 1] * image_width
    box_coords[:, 2] = boxes[:, 2] * image_height
    box_coords[:, 3] = boxes[:, 3] * image_width

    return box_coords


def TLTrim(image, box_coordinates):
    # return trimmed images containing all traffic light signals ahead: from
    # zero (no traffic lights) to whatever
    images = []
    for box in box_coordinates:
        x = int(np.round(box[1]))
        y = int(np.round(box[0]))
        w = int(np.round(box[3] - box[1]))
        h = int(np.round(box[2] - box[0]))
        trimmed_image = image[y:y + h, x:x + w]

        # return trimmed_image
        # cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/trimmed_{}.png'.format(time.time()),trimmed_image)
        images.append(trimmed_image)

    return images


def TLImage_Pro(image):
    # Image processing using openCV
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/HSV_{}.png'.format(time.time()),image_hsv)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    red1 = cv2.inRange(image_hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    red2 = cv2.inRange(image_hsv, lower_red, upper_red)
    converted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)
    # cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/converted_{}.png'.format(time.time()),converted_img)
    blur_img = cv2.GaussianBlur(converted_img, (15, 15), 0)

    circles = cv2.HoughCircles(blur_img, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70,
                               param2=30, minRadius=5, maxRadius=120)
    # cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/circles_{}.png'.format(time.time()),circles)
    return circles


'''
with open('logfile.txt','wb') as f:
    #creates logfile from scratch
    f.write('new log file \n')
'''


detection_graph = load_graph(GRAPH_FILE)

image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

sess = tf.Session(graph=detection_graph)


class TLClassifier(object):
    def __init__(self):
        print("Classifier: initialized")

    def get_classification(self, image, work_mode):

        if work_mode == "simulator":
            img_circles = TLImage_Pro(image)
            if img_circles is not None:
                return TrafficLight.RED

            else:
                return TrafficLight.UNKNOWN

        # SSD with Image Processing
        elif work_mode == "site":

            # Blur the image so it seems more realistic and the classificator performs better
            gbeq_image = cv2.GaussianBlur(image, (5, 5), 0)
            boxes, scores, classes = TLDetection(gbeq_image, sess)
            # log('CAMERA IMAGE PROCESSED, {} boxes detected'.format(len(boxes)))
            boxes, scores, classes = TLBoxes(BOXES_SCORE_MIN, boxes, scores, classes)

            image_height = image.shape[0]
            image_width = image.shape[1]
            box_coordinates = TLResizeBoxes(boxes, image_height, image_width)
            # trimmed_lights = TLTrim(image, box_coordinates)

            if len(boxes) != 0:
                print("found boxes with prob > 0.5: ", boxes)
                most_common = np.argmax(np.bincount(classes))

                if most_common == 1:
                    print("Red!")
                    return TrafficLight.RED

                elif most_common == 2:
                    print("Yellow")
                    return TrafficLight.YELLOW
                elif most_common == 3:
                    print("Green")
                    return TrafficLight.GREEN
                else:
                    print("Unknown")
                    return TrafficLight.UNKNOWN
            else:
                return TrafficLight.UNKNOWN

        else:
            print("wrong working mode, the model only works with simulator or site")
            return None
