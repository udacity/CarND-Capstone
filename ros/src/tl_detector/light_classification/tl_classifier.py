from styx_msgs.msg import TrafficLight
import tensorflow as tf
from tensorflow import saved_model
import numpy as np

class TLClassifier(object):
        
    def __init__(self, sess=None, is_site=None, classify=False):
        if classify:
        	model_path = './saved_model/'
        	if is_site:
            		model_path = './saved_real_model/'
        	self.sess = sess
        	saved_model.loader.load(sess, [saved_model.tag_constants.SERVING], model_path)
        	graph = tf.get_default_graph()
        	self.input_layer = graph.get_tensor_by_name('input:0')
        	self.output = graph.get_tensor_by_name('output:0')
        	self.light_map = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN, TrafficLight.UNKNOWN]
    	else:
		pass
    def get_classification(self, image, classify=False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if classify:
        	imgs = np.array([image])/255.0
        	output = self.sess.run([self.output], feed_dict={self.input_layer: imgs})
        	pred = np.argmax(output[0], axis=1)[0]
        	return self.light_map[pred]
	else:
		return TrafficLight.UNKNOWN
    
