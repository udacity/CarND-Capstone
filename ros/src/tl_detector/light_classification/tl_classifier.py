from styx_msgs.msg import TrafficLight
import rospy
import numpy as np
import tensorflow as tf

class TLClassifier(object):
    def __init__(self, model_file):
        #TODO load classifier
        self.model = model_file

        self.detection_graph = tf.Graph()
        self.min_score_threshold = 0.5
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                rospy.loginfo("loaded graph from frozen model")
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')

        config = tf.ConfigProto()
        # JIT level, this can be set to ON_1 or ON_2
        jit_level = tf.OptimizerOptions.ON_1
        config.graph_options.optimizer_options.global_jit_level = jit_level
        self.sess = tf.Session(config=config, graph=self.detection_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        light_state = TrafficLight.UNKNOWN

        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                    [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                    feed_dict={self.image_tensor: img_expanded})
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > self.min_score_threshold:
                    if classes[i] == 1:
                        rospy.loginfo('Light Detected: GREEN')
                        light_state = TrafficLight.GREEN
                    elif classes[i] == 2:
                        rospy.loginfo('Light Detected: RED')
                        light_state = TrafficLight.RED
                    elif classes[i] == 3:
                        rospy.loginfo('Light Detected: YELLOW')
                        light_state = TrafficLight.YELLOW
                    else:
                        rospy.loginfo('No Light Detected')

        return light_state
