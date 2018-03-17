# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:16:48 2018

@author: Danilo Canivel
"""
from styx_msgs.msg import TrafficLight
import rospy
import cv2
import numpy as np
import tensorflow as tf
from utils import label_map_util
from utils import visualization_utils as vis_util
import sys
from matplotlib import pyplot as plt
import time

sys.path.append('light_classification')

DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
PATH_TO_CKPT_SIM = 'light_classification/frozen_graphs/frozen_inference_graph_sim.pb'
PATH_TO_CKPT_REAL = 'light_classification/frozen_graphs/frozen_inference_graph_real.pb'
PATH_TO_LABELS = 'light_classification/label_map.pbtxt'
NUM_CLASSES = 4

class TLClassifier(object):

    def __init__(self, for_real=False):
        if for_real:
            frozen_inf = PATH_TO_CKPT_REAL
        else:
            frozen_inf = PATH_TO_CKPT_SIM

        # self.category_index = {1: {'id': 1, 'name': 'Red'},
        #                        2: {'id': 2, 'name': 'Yellow'},
        #                        3: {'id': 3, 'name': 'Green'},
        #                        4: {'id': 4, 'name': 'Unknown'}}

        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map,
                                                                         max_num_classes=NUM_CLASSES,
                                                                         use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

        self.detection_graph = tf.Graph()

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # for JIT optimization
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(frozen_inf, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # Definite input and output Tensors for detection_graph
            # Each box represents a part of the image where a particular object was detected.
            # return the level of confidence for each of the box detects.
            self.session = tf.Session(graph=self.detection_graph)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def prepare_trafficlight(self, image):
        #print(image)
        # image = cv2.cvtColor(cv2.imread(image), cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (24, 72))
        return np.divide(image, 255).astype(np.float32)

    def save_origin_image(self, image_np, boxes, classes, scores, category_index, label_state):

        # Size of the output images.
        IMAGE_SIZE = (12, 8)
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            min_score_thresh=.5,
            use_normalized_coordinates=True,
            line_thickness=3)
        plt.figure(figsize=IMAGE_SIZE)
        plt.imshow(image_np)

        # save augmented images into hard drive
        plt.savefig('output_imgs/'+ label_state + '_' + str(time.time()) +'.png')

    def get_category(self, categories, index):
        for category in categories:
            if category['id'] == index:
                return category
        return None
    
    def get_classification(self, image, save_tl=False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            uint8 GREEN=2
            uint8 YELLOW=1
            uint8 RED=0

        """

        # reshape to [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)
        # Detect
        start_time = time.time()
        (boxes, scores, classes, num) = self.session.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        rospy.logdebug("Detection ms = %s", (time.time() - start_time) * 1000.0)

        start_time = time.time()

        score_thresh = scores[0][0]
        if score_thresh < 0.5:
            state = TrafficLight.UNKNOWN
        else:
            class_index = int(classes[0][0])
            category = self.get_category(self.categories, class_index)
            if category is not None:
                if (class_index == 0):
                    state = TrafficLight.UNKNOWN
                elif (class_index == 1):
                    state = TrafficLight.RED
                elif (class_index == 2):
                    state = TrafficLight.YELLOW
                else:
                    state = TrafficLight.GREEN

        if save_tl:

            if (state == 0):
                label_state = 'RED'
            elif (state == 1):
                label_state = 'YELLOW'
            elif (state == 2):
                label_state = 'GREEN'
            else:
                label_state = 'UNKNOWN'

            self.save_origin_image(image, boxes, classes, scores, self.category_index, label_state)

        return state

    def get_classification_batch_argmax(self, image_list, save_tl=False):
            """Determines the color of the traffic light in an batch of images
    
            Args:
                image_list (cv::Mat): list of images containing the traffic light
    
            Returns:
                int: argmax of the IDs of traffic light color (specified in styx_msgs/TrafficLight)
                uint8 GREEN=2
                uint8 YELLOW=1
                uint8 RED=0
    
            """
            stateAcc = [0, 0, 0, 0]
            imgs_preps = [self.prepare_trafficlight(image=i) for i in image_list]
            for image in imgs_preps:
                # reshape to [1, None, None, 3]
                image_np_expanded = np.expand_dims(image, axis=0)
                # Detect
        
                start_time = time.time()
                boxes, scores, classes, num = self.session.run([self.detection_boxes,
                                                                self.detection_scores,
                                                                self.detection_classes,
                                                                self.num_detections],
                                                                feed_dict={self.image_tensor: image_np_expanded})
                
                print("Detection ms = ", (time.time() - start_time) * 1000.0)

                score_thresh = scores[0][0]
                print(score_thresh)
                if score_thresh < 0.5:
                    state = 4  # TrafficLight.UNKNOWN
                    stateAcc[3]+= 1
                    print("TLC Unknown hit")
                else:
                    class_index = int(classes[0][0])
                    print(self.categories)
                    print(class_index)
                    category = self.get_category(self.categories, class_index)
                    if category is not None:
                        print("%s, %s, %s", category['name'], score_thresh, class_index)
        
                        if class_index == 0:
                            state = 4  # TrafficLight.UNKNOWN
                            stateAcc[3]+= 1
                            print("TLC Unknown hit")
                        elif class_index == 1:
                            state = 0  # TrafficLight.RED
                            stateAcc[0]+= 1
                            print("TLC Red hit")
                        elif class_index == 2:
                            state = 1  # TrafficLight.YELLOW
                            stateAcc[1]+= 1
                            print("TLC Yellow hit")
                        else:
                            state = 2 # TrafficLight.GREEN
                            stateAcc[2]+= 1
                            print("TLC Green hit")
        
                if save_tl:
        
                    if state == 0:
                        label_state = 'RED'
                    elif state == 1:
                        label_state = 'YELLOW'
                    elif state == 2:
                        label_state = 'GREEN'
                    else:
                        label_state = 'UNKNOWN'
        
                    self.save_origin_image(image, boxes, classes, scores, self.category_index, label_state)
                    
            max_index = stateAcc.index(max(stateAcc))
    
            return max_index