from styx_msgs.msg import TrafficLight
from keras.preprocessing.image import load_img, img_to_array
import cv2
import numpy as np
import keras.backend as K
import tensorflow as tf
from train import SqueezeNet
from consts import IMAGE_WIDTH, IMAGE_HEIGHT

class TLClassifier(object):
    def __init__(self, sim):
        K.set_image_dim_ordering('tf')

        self.ready = False

        if sim:
            model_name = 'squeezeNet_sim'
        else:
            model_name = 'squeezeNet_real'

        with tf.gfile.GFile('light_classification/{}.optimized.pb'.format(model_name), 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        self.graph = tf.Graph()
        self.sess = tf.Session(graph = self.graph)

        with self.graph.as_default() as graph:
            tf.import_graph_def(graph_def)

            # Restoring checkpoint weights fails with error.
            # saver = tf.train.Saver()
            # saver.restore(self.sess, 'light_classification/{}.ckpt'.format(model_name))

        for op in self.graph.get_operations():
            print(op.name)

        self.ready = True

        self.pred_dict = {0: TrafficLight.UNKNOWN,
                          1: TrafficLight.RED,
                          2: TrafficLight.GREEN}

        self.debug_print = True

    def get_classification(self, image):
        pred, preds = self.get_classification_detailed(image)
        return pred

    def get_classification_detailed(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        preds = None
        if self.ready:
            # SqueezeNet model expects RGB image input.
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))
            image = image.astype(K.floatx())
            image /= 255.0
            image = np.expand_dims(image, axis=0)

            with self.graph.as_default() as graph:
                learning_phase_tensor = self.graph.get_tensor_by_name('import/fire9_dropout/keras_learning_phase:0')
                op_tensor = self.graph.get_tensor_by_name('import/softmax/Softmax:0')
                input_tensor = self.graph.get_tensor_by_name('import/input_1:0')
                feed_dict = {input_tensor: image, learning_phase_tensor: False}
                preds = self.sess.run(op_tensor, feed_dict)

            pred_index = np.argmax(preds)
            pred = self.pred_dict[pred_index]
            if self.debug_print:
                print('TLClassifier', friendly_name(pred), preds)
        else:
            pred = TrafficLight.UNKNOWN

        return (pred, preds)

def friendly_name(pred):
    pred_name_dict = {TrafficLight.UNKNOWN: "Unknown",
                      TrafficLight.RED: "Red",
                      TrafficLight.GREEN: "Green"}

    return pred_name_dict[pred]
