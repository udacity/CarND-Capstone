from styx_msgs.msg import TrafficLight
from tl_detection.detector import traffic_light_detector
from keras.models import load_model
import h5py
import numpy as np
from keras import __version__ as keras_version
import tensorflow as tf
import cv2
import rospy

class TLClassifier(object):
    def __init__(self,detector_mdl_path,classifier_mdl):
        
        # Load tl detector
        self.tld = traffic_light_detector(detector_mdl_path)

        # Load tl classifier
        f = h5py.File(classifier_mdl, mode='r')
        model_version = f.attrs.get('keras_version')
        Keras_version = str(keras_version).encode('utf8')

        if model_version != keras_version:
            rospy.loginfo('You are using Keras version ', keras_version,
                  ', but the model was built using ', model_version)

        self.simulator_model = load_model(classifier_mdl)
        self.simulator_model._make_predict_function()
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Detect bounding boxes
        box_coords, _ = self.tld.predict(image)
        
        if len(box_coords) == 0:
            rospy.loginfo('No boxes detected')
            return TrafficLight.UNKNOWN
        
        # Identify light state
        num_detected = [0] * 3 # count how many each light detected in case not all boxes agree
        
        for box in box_coords:
            x1 = int(box[0])
            y1 = int(box[1])
            x2 = int(box[2])
            y2 = int(box[3])

            tl_img = image[x1:x2,y1:y2]
            dsize = (15, 30)
            tl_img = cv2.resize(tl_img, dsize)

            image_array = np.asarray(tl_img)

            with self.graph.as_default():
                labels = self.simulator_model.predict(image_array[None,:,:,:])
                predict = np.argmax(labels)
                
            num_detected[predict] += 1
        
        predict = num_detected.index(max(num_detected))
        rospy.loginfo('%d boxes detected. '%(len(box_coords))
                      +'Each light detected (%d,%d,%d) times. '%(num_detected[0],num_detected[1],num_detected[2])
                      +'Return prediction is %d.'%predict)

        return predict
