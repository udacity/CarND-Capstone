import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        model_path = rospy.get_param("/traffic_light_model")

        self.sess = tf.Session()
        saver = tf.train.import_meta_graph(model_path + '.meta')
        saver.restore(self.sess, model_path)
        graph = tf.get_default_graph()
        self.input_operation = graph.get_operation_by_name('Placeholder')
        self.output_operation = graph.get_operation_by_name('final_result')

    def predict(self, image):
        image = cv2.resize(image, (224, 224))
        image = image / 255.
        image = image.reshape((1, 224, 224, 3))

        results = self.sess.run(self.output_operation.outputs[0], {
            self.input_operation.outputs[0]: image
        })
        results = np.squeeze(results)
        return results.argsort()[-1]

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        return {
            0: TrafficLight.GREEN,
            1: TrafficLight.RED,
            2: TrafficLight.UNKNOWN,
            3: TrafficLight.YELLOW,
        }.get(self.predict(image), TrafficLight.UNKNOWN)
