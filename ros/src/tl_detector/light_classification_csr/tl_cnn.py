import cv2
import numpy as np
import tensorflow as tf
import rospy

from time import time
from keras.models import model_from_json
from keras.preprocessing.image import img_to_array

from styx_msgs.msg import TrafficLight


class TrafficLightModel(object):
    """
    Keras Model:

        input_shape = (150, 150, 3)
        model = Sequential()
        model.add(Conv2D(32, 3, 3, input_shape=input_shape))
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Conv2D(32, 3, 3))
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Conv2D(64, 3, 3))
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Flatten())
        model.add(Dense(64))
        model.add(Activation('relu'))
        model.add(Dropout(0.5))
        model.add(Dense(4))
        model.add(Activation('softmax'))
    """

    idx_to_class = {
        0: TrafficLight.GREEN,
        1: TrafficLight.RED,
        2: TrafficLight.UNKNOWN,
        3: TrafficLight.YELLOW,
    }

    idx_to_string = {
        0: 'GREEN',
        1: 'RED',
        2: 'UNKNOWN',
        3: 'YELLOW',
    }

    idx_to_color = {
        0: (0, 255, 0),
        1: (255, 0, 0),
        2: (0, 0, 0),
        3: (244, 219, 36),
    }

    def __init__(self):
        # Load Model
        fname = './light_classification_csr/model_keras_v1.2.0_8840_16_100'
        file = open(fname + '.json', 'r')
        json_string = file.read()
        file.close()
        self.model = model_from_json(json_string)
        self.model.load_weights(fname + '.h5')

        # Due to bug in Keras
        # https://github.com/fchollet/keras/issues/2397
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

        rospy.loginfo("CNN model loaded")

    def predict(self, image):

        # Pipeline start time
        start = time()

        # Change color format and resize to 150x150
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(image, (150, 150))

        # Normalized numpy array with shape (150, 150, 3)
        x = img_to_array(resized, dim_ordering='tf') / 255.

        # Numpy array with shape (1, 150, 150, 3)
        x = x.reshape((1,) + x.shape)

        with self.graph.as_default():
            prediction = self.model.predict(x)

        pred_idx = np.argmax(prediction[0])

        result = self.idx_to_class[pred_idx]

        # Pipeline end time
        end = time()

        rospy.loginfo('%s in %4.2fms' % (self.idx_to_string[pred_idx],
                                         int((end - start) * 1000)))

        # Debug
        cv2.rectangle(resized, (0, 0), (150, 10),
                      self.idx_to_color[pred_idx], 10)
        resized = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', resized)
        cv2.waitKey(1)

        return result
