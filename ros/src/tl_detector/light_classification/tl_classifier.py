from styx_msgs.msg import TrafficLight
import tensorflow as tf
import time
import numpy as np
import rospy


class TLClassifier(object):

    def __init__(self, model_path, is_site):
        self.model_path = model_path
        self.load_detection_model()
        self.score_thresh = .3
        # the postprocess of the site and simulator is different
        self.is_site = is_site


    def load_detection_model(self):
        """
        load a frozen traffic light detection tensorflow model into memory
        :return:
        """

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # limit the memory for model to use
            config = tf.ConfigProto()
            # config.gpu_options.allow_growth = True
            config.gpu_options.per_process_gpu_memory_fraction = 0.6

            self.sess = tf.Session(graph=self.detection_graph, config=config)

            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_expanded})

        if self.is_site:
            result = self.only_threshold(classes[0], scores[0])
        else:
            result = self.threshold_and_vote(classes[0], scores[0])
        traffic_id = self.result_to_traffic_id(result)

        return traffic_id

    def only_threshold(self, classes, scores):
        c = classes[0]
        s = scores[0]
        if c == 2:
            # the class is RED
            if s > 0.2:
                return c
            else:
                return TrafficLight.UNKNOWN
        elif c == 1 or c == 3:
            # the class is GREEN or YELLOW
            if s > 0.3:
                return c
            else:
                return TrafficLight.UNKNOWN
        else:
            return TrafficLight.UNKNOWN

    def threshold_and_vote(self, classes, scores):
        # only look for the top 4 score
        vote = []
        for i in range(3):
            if scores[i] > self.score_thresh:
                vote.append(classes[i])
        if vote:
            # return the most common item
            return max(vote, key=vote.count)
        else:
            # return 4
            return TrafficLight.UNKNOWN

    def traffic_id_to_name(self, traffic_id):
        traffic_light_names = ['RED', 'YELLOW', 'GREEN', 'ERROR', 'UNKNOWN']
        return traffic_light_names[traffic_id]

    def result_to_traffic_id(self, result):
        result = int(result - 1)
        if self.is_site:
            traffic_light_id = [2, 0, 1, 4]
        else:
            traffic_light_id = [0, 1, 2, 4]
        return traffic_light_id[result]


# if __name__ == '__main__':
#     model = '../model/frozen_inference_graph_sim.pb'
#
#     tl_classifier = TLClassifier(model)
#     import cv2
#     img = cv2.imread('/devdisk/Udacity-Project/script/sim_data/test/1522226416.92_2.png')
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     tl_classifier.get_classification(img)
