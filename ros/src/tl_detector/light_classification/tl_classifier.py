from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy
import yaml

class TLClassifier(object):
    def __init__(self):

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
	self.is_sim = self.config["is_sim"]

        if self.is_sim:
            PATH_TO_GRAPH = r'light_classification/model/ssd_sim/frozen_inference_graph.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/model/ssd_udacity/frozen_inference_graph.pb'

        self.graph = tf.Graph()
        self.threshold = .5


        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            try: 
                with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                    od_graph_def.ParseFromString(fid.read())
                    tf.import_graph_def(od_graph_def, name='')
            except EnvironmentError:
                rospy.logfatal("Can't load model:".format(PATH_TO_GRAPH))

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            #rospy.logdebug(c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        
        #rospy.logdebug ('SCORE: {} Class: {}'.format(scores[0],classes[0]))
        state = TrafficLight.UNKNOWN
        if scores[0] > self.threshold:
            if classes[0] == 1:
                state = TrafficLight.GREEN
            elif classes[0] == 2:
                state = TrafficLight.RED
            elif classes[0] == 3:
                state = TrafficLight.YELLOW
            rospy.loginfo ('Detected. SCORE: {} Class: {}'.format(scores[0],classes[0]))
        else:
            rospy.loginfo ('Undetected. SCORE: {} Class: {}'.format(scores[0],classes[0]))
        return state
