import numpy as np
import PIL, cv2
import light_helper as Helper
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, path='light_classifier_model.h5'):
        self.model = load_model(path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        original = cv2.resize(image, (800, 600), interpolation=cv2.INTER_CUBIC)
        processed = Helper.preprocess(original)

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
                # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

                # Height expanded.
                h = h * 7
                y = y - h*0.25
                # Width expanded.
                x = (x+w/2) - h/4
                w = h/2
                # Crop
                x, y, w, h = int(max(0,x)), int(max(0,y)), int(w), int(h)
                image = original[y:(y+h), x:(x+w)]
                cv2.imwrite('test.jpg', image)
                has_red = (self.predict(image) == TrafficLight.RED) or has_red

                # cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),1)
                # cv2.putText(img, RESULT[predict(image)], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255))

        prediction = TrafficLight.RED if has_red else TrafficLight.UNKNOWN
        return prediction

    def predict(self, image):
        if type(image) == np.ndarray:
            image_array = cv2.resize(image, (64, 64), interpolation=cv2.INTER_CUBIC)
            image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        else:
            image_array = img_to_array(image.resize((64, 64), PIL.Image.ANTIALIAS))
        prediction = self.model.predict(image_array[None, :])
        if prediction[0][0] == 1:
          return TrafficLight.GREEN
        elif prediction[0][1] == 1:
          return TrafficLight.RED
        else:
          return TrafficLight.UNKNOWN
