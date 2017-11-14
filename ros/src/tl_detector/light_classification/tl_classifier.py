from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self, model_path):
        #TODO load classifier
	self.tf_session = None
        self.predict = None
        self.model_path = model_path
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	if self.tf_session is None:
        # get the traffic light classifier
		self.config = tf.ConfigProto(log_device_placement=True)
           	self.config.gpu_options.per_process_gpu_memory_fraction = 0.2  # don't hog all the VRAM!
            	self.config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
		self.tf_session = tf.Session(config=self.config)
            	self.saver = tf.train.import_meta_graph(self.model_path + '/checkpoints/generator.ckpt.meta')
            	self.saver.restore(self.tf_session, tf.train.latest_checkpoint(self.model_path + '/checkpoints/'))
	
	predict = [ TrafficLight.RED ]
        if self.predict is not None:
            predict = self.tf_session.run(self.predict, feed_dict = {
                self.input_real: self.scale(image.reshape(-1, 600, 800, 3)),
                self.drop_rate:0.})

        return int(predict[0])


