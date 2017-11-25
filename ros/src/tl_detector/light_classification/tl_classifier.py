from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import os

class TLClassifier(object):
    def __init__(self, model_path):
        #TODO load classifier
        self.tf_session = None
        self.predict = None
        self.model_path = model_path

    	if self.tf_session is None:
            # get the traffic light classifier
            self.config = tf.ConfigProto(log_device_placement=True)
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.2  # don't hog all the VRAM!
            self.config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
            self.tf_session = tf.Session(config=self.config)
            #self.saver = tf.train.import_meta_graph(self.model_path + '/checkpoints/generator.ckpt.meta')
            #print("try os getcwd:",os.getcwd())
            self.saver = tf.train.import_meta_graph(os.getcwd()+'/../../../training/Simulator/checkpoints/generator.ckpt.meta')
            self.saver.restore(self.tf_session, tf.train.latest_checkpoint(os.getcwd()+'/../../../training/Simulator/checkpoints'))

            #get tensorflow by name
            self.tf_graph = tf.get_default_graph()
            self.input_real = self.tf_graph.get_tensor_by_name("input_real:0")
            self.drop_rate = self.tf_graph.get_tensor_by_name("drop_rate:0")
            self.predict = self.tf_graph.get_tensor_by_name("predict:0")

            print("\nClassifier initialization done.\nPlease start the simulator.")

    # scale the image features from -1 to 1 for the classifier
    def scale(self, x, feature_range=(-1, 1)):
        """Rescale the image pixel values from -1 to 1

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            image (cv::Mat): image rescaled from -1 to 1 pixel values

        """
        # scale to (-1, 1)
        x = ((x - x.min())/(255 - x.min()))

        # scale to feature_range
        min, max = feature_range
        x = x * (max - min) + min
        return x


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        predict = [ TrafficLight.UNKNOWN ]
        if self.predict is not None:
            predict = self.tf_session.run(self.predict, feed_dict = {
                self.input_real: self.scale(image.reshape(-1, 600, 800, 3)),
                self.drop_rate:0.})

        #print("length of predict:", len(predict),"predict[0]:",predict[0])

        return int(predict[0])
