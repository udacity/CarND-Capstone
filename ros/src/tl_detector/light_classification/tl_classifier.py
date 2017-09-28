import numpy as np
import PIL, cv2
import tensorflow as tf
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, path='light_classifier_model.h5'):
        self.model = load_model(path)
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        processed = self.preprocess(image)

        # ----- Filter contour.
        width, height = processed.shape
        _, contours, _= cv2.findContours(processed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        has_red = False
        for jdx, contour in enumerate(contours):
            # Check ratio and size.
            x,y,w,h = cv2.boundingRect(contour)
            ok_ratio = (h / w > 0.85) and (h / w < 1.4)
            ok_size = (w > 5) and (h > 5)

            if ok_ratio and ok_size:
                # Height expanded.
                h = h * 7
                y = y - h*0.25
                # Width expanded.
                x = (x+w/2) - h/4
                w = h/2
                # Crop
                x, y, w, h = int(max(0,x)), int(max(0,y)), int(w), int(h)
                target = image[y:(y+h), x:(x+w)]
                # Model classification.
                has_red = (self.predict(target) == TrafficLight.RED) or has_red

        return TrafficLight.RED if has_red else TrafficLight.UNKNOWN

    def preprocess(self, input_img):
        # img = cv2.resize(input_img, (800, 600), interpolation=cv2.INTER_CUBIC)
        # out = cv2.bilateralFilter(img,11,75,75)
        out = input_img
        # ----- RGB
        rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

        # Filter BLUE color
        rgb[ rgb[:,:,2] > 180 ] = 0
        # Filter YELLOW color
        rgb[ (rgb[:,:,1] < 230) & (rgb[:,:,2] > 150) ] = 0
        # Filter RED < color
        rgb[ rgb[:,:,0] < rgb[:,:,1] ] = 0
        rgb[ rgb[:,:,0] < rgb[:,:,2] ] = 0
        # Filter EQUAL channel color
        mask1 = (rgb[:,:,0]-rgb[:,:,1] < 20) | (rgb[:,:,1]-rgb[:,:,0] < 20)
        mask2 = (rgb[:,:,0]-rgb[:,:,2] < 20) | (rgb[:,:,2]-rgb[:,:,0] < 20)
        mask3 = (rgb[:,:,1]-rgb[:,:,2] < 20) | (rgb[:,:,2]-rgb[:,:,1] < 20)
        rgb[ mask1 & mask2 & mask3 ] = 0
        rgb[ mask2 ] = 0

        # ----- Combine channel
        image = rgb[:,:,0]
        _, image = cv2.threshold(image,235,255,cv2.THRESH_BINARY)

        return image

    def predict(self, image):
        if image is not None:
            image_array = cv2.resize(image, (64, 64), interpolation=cv2.INTER_CUBIC)
            image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)

            # Make prediction from model.
            with self.graph.as_default():
                prediction = self.model.predict(image_array[None, :])

            # Process prediction.
            if prediction[0][0] == 1:
              return TrafficLight.GREEN
            elif prediction[0][1] == 1:
              return TrafficLight.RED
            else:
              return TrafficLight.UNKNOWN
        return TrafficLight.UNKNOWN
