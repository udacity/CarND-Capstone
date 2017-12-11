import rospy
from styx_msgs.msg import TrafficLightState
import numpy as np
from keras.models import model_from_json
from keras import backend as K
import cv2

class TLClassifier(object):
    def __init__(self):

        # clear TF graph state
        K.clear_session()

        model_arch_path = 'keras_model/squeezenet_transfer_learned_architecture.json'
        model_weights_path = 'keras_model/squeezenet_transfer_learned_weights.h5'

        # load keras model architecture
        json_file = open(model_arch_path)
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        self.model._make_predict_function()

        # load weights into model
        self.model.load_weights(model_weights_path)
        rospy.logwarn('Perception keras model loaded')

        #self.frame_count = 0

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLightState)

        """

        #self.frame_count = self.frame_count + 1

        # resize OpenCV image to (227, x)
        org_width, org_height = (600.0, 800.0)
        resize_width, resize_height = (227.0, 227.0)
        scale = resize_height / org_width
        resized_img = cv2.resize(image, (0,0), fx=scale, fy=scale)

        # turn resized image to numpy array
        cropped_img = np.asarray(resized_img[:,:])

        # crop image to (227,227) image
        cropped_img = cropped_img[0:227, 0:227]

        # for debugging
        # cv2.imwrite('/tmp/imgs/{}_org.png'.format(self.frame_count), image)
        # cv2.imwrite('/tmp/imgs/{}_resized.png'.format(self.frame_count), resized_img)
        # cv2.imwrite('/tmp/imgs/{}_cropped.png'.format(self.frame_count), cropped_img)

        img = np.array([cropped_img])
        prediction = self.model.predict(img)[0]

        prediction_labels = [TrafficLightState.GREEN, TrafficLightState.RED, TrafficLightState.YELLOW, TrafficLightState.UNKNOWN]
        labels_names = ['GREEN', 'RED', 'YELLOW', 'UNKNOWN']

        light_state = prediction_labels[prediction.argmax()]

        rospy.logwarn('Traffic Light Prediction - ' + labels_names[prediction.argmax()])

        return light_state
