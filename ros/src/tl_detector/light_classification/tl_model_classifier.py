from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
from keras.models import load_model


class TLModelClassifier(object):
    def __init__(self):
        self.model12_loaded = load_model('./light_classification/model.h5')

    def cropAndReshape(img):
        img_arr = np.asarray(img)
        # Cut lower part (Engine hood)
        remaining = 80
        img2=(img_arr[0:int(img_arr.shape[0]/2+remaining),
                 0:img_arr.shape[1]]).copy()
    
        return cv2.resize(img2, (target_size, target_size))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        predicted = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.UNKNOWN]
        state = self.model12_loaded.predict(np.expand_dims(cropAndReshape(img), axis=0))
        idx = np.argmax(state)
        return predicted[idx]