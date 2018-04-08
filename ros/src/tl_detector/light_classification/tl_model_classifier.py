from styx_msgs.msg import TrafficLight
import rospy
import cv2
import h5py
import numpy as np
import keras
from keras.models import load_model


class TLModelClassifier(object):
    def __init__(self):
        # check that model Keras version is same as local Keras version
        path = './light_classification/model.h5'
        f = h5py.File(path, mode='r')
        model_version = f.attrs.get('keras_version')
        keras_version = str(keras.__version__).encode('utf8')

        if model_version != keras_version:
            rospy.loginfo('You are using Keras version {}, but the model was built using {}'.format(keras_version, model_version))
        else:
            rospy.loginfo('Both using Keras ver {}'.format(keras_version))
        self.model12_loaded = load_model(path)
        
        # Line below is to resolve https://github.com/keras-team/keras/issues/2397
        rospy.loginfo('testing model: {}'.format(self.model12_loaded.predict(np.zeros((1, 224, 224, 3)))))

    def cropAndReshape(self, img):
        img_arr = np.asarray(img)
        # Cut lower part (Engine hood)
        remaining = 80
        img2=(img_arr[0:int(img_arr.shape[0]/2+remaining),
                 0:img_arr.shape[1]]).copy()
    
        return cv2.resize(img2, (224, 224)) #224 x 224 for vgg

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        predicted = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.UNKNOWN]
        processed_img = self.cropAndReshape(image)
        expanded = np.expand_dims(processed_img, axis=0)
        state = self.model12_loaded.predict(expanded)
        idx = np.argmax(state)
        rospy.loginfo("\nTrafficLight.GREEN: 0 TrafficLight.RED: 1 TrafficLight.YELLOW: 2 TrafficLight.UNKNOWN: \n\n Detected {}".format(idx))
        return predicted[idx]