# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:16:48 2018

@author: Danilo Canivel
"""
#from styx_msgs.msg import TrafficLight
import pickle
import cv2
import numpy as np
from gcforest.gcforest import GCForest
from gcforest.utils.config_utils import load_json

class TLClassifier(object):
    def __init__(self, clf_name):
        with open(clf_name, "rb") as f:
            self.gc = pickle.load(f)
            
    
    def prepare_trafficlight(self, image):
        #print(image)
        image = cv2.cvtColor(cv2.imread(image), cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (24, 72))
        return np.divide(image, 255).astype (np.float32)
    
    def get_classification_batch_argmax(self, image_list):
        """Determines the color of the traffic light in an batch of images

        Args:
            image_list (cv::Mat): list of images containing the traffic light

        Returns:
            int: argmax of the IDs of traffic light color (specified in styx_msgs/TrafficLight)
            uint8 GREEN=2
            uint8 YELLOW=1
            uint8 RED=0

        """
        imgs_preps = [self.prepare_trafficlight(image=i) for i in image_list]
        X_test = np.array(imgs_preps)
        y_pred = self.gc.predict(X_test)
        most_freq = np.bincount(y_pred).argmax()
        return most_freq
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            uint8 GREEN=2
            uint8 YELLOW=1
            uint8 RED=0

        """
        img_prep = [self.prepare_trafficlight(image=image)]
        X_test = np.array(img_prep)
        y_pred = self.gc.predict(X_test)
        return y_pred[0]
