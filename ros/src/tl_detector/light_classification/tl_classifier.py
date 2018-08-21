from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy

THRESHOLD = .5
class TLClassifier(object):
    def __init__(self, is_site=False):
        self.is_site = is_site
        # By default we will use sim model
        if self.is_site :
            rospy.logwarn("Using model for real car")
            model_path = "light_classification/model/frozen_inference_graph_real.pb"
        else:
            rospy.logwarn("Using model for sim car")
            model_path = "light_classification/model/frozen_inference_graph_sim.pb"

        rospy.logwarn("Initialize Graph & Session start")
        self.graph = tf.Graph()

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # TF Graph is ready
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.detect_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.detect_scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.detect_classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        # pass graph session to rest of program
        self.sess = tf.Session(graph=self.graph)
        rospy.logwarn("Initialize Graph & Session end")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        start_clock = datetime.datetime.now()
        with self.graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            boxes, scores, classes, num = self.sess.run(
                [self.detect_boxes, self.detect_scores, self.detect_classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})


        stop_clock = datetime.datetime.now()
        rospy.logwarn("Classified in {0}".format((stop_clock-start_clock).total_seconds()))

        # squeeze here from shape (1,10) to (,10)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        rospy.logwarn('SCORES: {0} CLASSES: {1}'.format(scores[0], classes[0]))

        # color code
        # {
        #  1: {'id': 1, 'name': 'Green'},
        #  2: {'id': 2, 'name': 'Red'},
        #  3: {'id': 3, 'name': 'Yellow'},
        #  4: {'id': 4, 'name': 'off'}
        # }
        if scores[0] >= THRESHOLD:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.RED
            elif classes[0] == 3:
                return TrafficLight.YELLOW
        return TrafficLight.UNKNOWN
