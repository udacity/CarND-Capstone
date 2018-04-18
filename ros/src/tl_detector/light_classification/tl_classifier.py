from styx_msgs.msg import TrafficLight
import keras
from keras.utils.generic_utils import CustomObjectScope
from keras.models import load_model
from keras.applications import resnet50
import rospy
import numpy as np
import cv2
import tensorflow as tf
from light_classification.models import model_file_join
import os.path


class TLClassifier(object):
    def __init__(self):
        self.current_light = TrafficLight.UNKNOWN

        # Load the Model
        model_path = "light_classification/models/"
        model_filename = "ResNet50-UdacityRealandSimMix-Best-val_acc-1.0.hdf5"
        model_file = model_path + model_filename
        if os.path.isfile(model_file):
            # If model file is already in place
            model = load_model(model_file)
        else:
            # Join the split model files before loading
            model_file_join.join_file(model_file, 4)
            model = load_model(model_file)

        self.model = model
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        np_final = np.zeros((1, 224, 224, 3))
        yhat = model.predict(np_final)

        self.labels = np.array(['green', 'none', 'red', 'yellow'])
        self.resize_width = 224
        self.resize_height = 224

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.resize(image, (self.resize_width, self.resize_height))
        np_image_data = np.asarray(image)
        np_final = np.expand_dims(np_image_data, axis=0)
#        np_final = np_final/255
        np_final = resnet50.preprocess_input(np_final.astype('float64'))
        t0 = rospy.Time.now()
        model = self.model
        with self.graph.as_default():
            yhat = model.predict(np_final)
        dt = rospy.Time.now() - t0
#        yhat = yhat / yhat.sum()
        yhat = yhat[0]
        y_class = yhat.argmax(axis=-1)
        labels = self.labels

        rospy.loginfo('%s (%.2f%%) : GPU time (s) : %f', labels[y_class],
                      yhat[y_class]*100, dt.to_sec())

        self.current_light = TrafficLight.UNKNOWN
        if (yhat[y_class] > 0.5):
            if y_class == 0:
                self.current_light = TrafficLight.GREEN
            elif y_class == 2:
                self.current_light = TrafficLight.RED
            elif y_class == 3:
                self.current_light = TrafficLight.YELLOW

        return self.current_light
