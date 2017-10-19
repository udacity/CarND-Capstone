import os
import rospy
import numpy as np
from math import *
import threading as t
import tensorflow as tf
import time

from styx_msgs.msg import TrafficLight
from include.model.KaNet import KaNet

class TLClassifier(object):
    def __init__(self):
        self.graph = tf.get_default_graph()

        # load params
        self.classes = rospy.get_param("~tl_classes")
        self.values = rospy.get_param("~tl_values")
        self.weights_file = rospy.get_param("~tl_weights_file")
        self.max_detections = rospy.get_param("~tl_max_detections")

        with self.graph.as_default():
            self.model = KaNet(len(self.classes), (None, None, 3), 1.0, 0)
            self.model.load_weights(self.weights_file, by_name=True)

        rospy.loginfo("[TL Classifier] -> Model Loaded!")

        self.busy = False
        self.state = TrafficLight.UNKNOWN
    
    def map_label(self, output):
        """ Maps the argmax of model output to the traffic light label """
        val = self.values[output]

        return np.uint8(val)
    
    def get_prediction(self, images):

        with self.graph.as_default():
            predictions = []
            
            
            msize = min(self.max_detections, len(images))
            times = np.zeros(msize)

            for i in range(msize):
                t0 = time.time()
                img = np.asarray(images[i])

                rospy.loginfo("[TL Classifier] -> Input shape: " + str(img.shape))

                pred = self.model.predict(img[None, :, :, 0:3], batch_size=1)[0]

                rospy.loginfo("[TL Classifier] -> Prediction: " + str(pred))

                predictions.append(pred)
                
                t1 = time.time()
                times[i] = (t1 - t0) * 1000
            
            predictions = np.sum(predictions, axis=0, keepdims=True)
            pred_idx = np.argmax(predictions)
            
            self.state = self.map_label(pred_idx)
            rospy.loginfo("[TL Classifier] -> TL Predicted: " + self.classes[pred_idx] + ", in " + str(np.sum(times)) + "ms")

            self.busy = False
            
    
    def get_classification(self, images):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.busy:
            return TrafficLight.UNKNOWN

        self.busy = True

        thread = t.Thread(target = self.get_prediction, args=(images,))
        thread.start()
        thread.join()

        return self.state
