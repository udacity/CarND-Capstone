from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime

from PIL import Image


class TLClassifier(object):
    """
    This class uses the trained model from the TensorFlow Object Detection API.
    https://github.com/tensorflow/models/tree/master/research/object_detection

    For the Capstone Project, a Single Shot Detector with lightweight MobileNet
    Backbone was trained on the Bosch Traffic Light Dataset, LISA Traffic Light Dataset and
    Udacity Simulator and Site images.

    The Inference Code was adapted from:
    https://github.com/tensorflow/models/blob/master/research/object_detection/inference/detection_inference.py

    """
    def __init__(self):
        # Threshold for detections
        self.detection_threshold = 0.5

        # Path to frozen Graph
        graph_path = 'src/tl_detector/light_classification/inference_graph/frozen_inference_graph.pb'

        # Create the TensorFlow session in which the graph is loaded
        self.session = tf.Session()

        # Create Tensors for results
        self.num_detections_tensor = None
        self.detected_boxes_tensor = None
        self.detected_scores_tensor = None
        self.detected_labels_tensor = None
        self.image_tensor = None

        # Load the trained and frozen model graph with respective weights
        with self.session.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

                g = tf.get_default_graph()

                # Remember all the tensors we will need for inference
                # Most important: input image tensor:
                self.image_tensor = g.get_tensor_by_name('image_tensor:0')

                self.num_detections_tensor = tf.squeeze(g.get_tensor_by_name('num_detections:0'), 0)
                self.num_detections_tensor = tf.cast(self.num_detections_tensor, tf.int32)

                self.detected_boxes_tensor = tf.squeeze(g.get_tensor_by_name('detection_boxes:0'), 0)
                self.detected_boxes_tensor = self.detected_boxes_tensor[:self.num_detections_tensor]

                self.detected_scores_tensor = tf.squeeze(g.get_tensor_by_name('detection_scores:0'), 0)
                self.detected_scores_tensor = self.detected_scores_tensor[:self.num_detections_tensor]

                self.detected_labels_tensor = tf.squeeze(g.get_tensor_by_name('detection_classes:0'), 0)
                self.detected_labels_tensor = tf.cast(self.detected_labels_tensor, tf.int64)
                self.detected_labels_tensor = self.detected_labels_tensor[:self.num_detections_tensor]

    def get_classification(self, image):
        """
        Determines the color of the traffic light in the image by
        using the TensorFlow graph.

        We run the operations that will give us the boxes, scores and labels.
        Then we filter out the most probable scores (> threshold) and use the
        biggest box, since this will be the nearest traffic light.

        The graph will give us the following IDs :
        4: NA
        3: green
        2: yellow
        1: red

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        traffic_light_id = 4  # 4 equals to unknown

        id_mapping = {4: TrafficLight.UNKNOWN,
                      3: TrafficLight.GREEN,
                      2: TrafficLight.YELLOW,
                      1: TrafficLight.RED}

        results = []
        with self.session.graph.as_default():
            boxes, scores, labels = self.session.run([self.detected_boxes_tensor,
                                                      self.detected_scores_tensor,
                                                      self.detected_labels_tensor],
                                                     feed_dict={self.image_tensor: image})
            # Filter for probability (score)
            for i, score in enumerate(scores):
                if score > self.detection_threshold:
                    results.append({'box': boxes[i],
                                    'score': score,
                                    'id': labels[i]})

        # The boxes are encoded as xmin, xmax, ymin, ymax with normalized coordinates [0..1].
        # So lets find just the biggest box and take the traffic light state from it.
        max_bb_size = 0
        max_index = None
        for idx, bb in enumerate(results):
            bb_size = (bb['box'][1] - bb['box'][0]) * (bb['box'][3] - bb['box'][2])
            if max_bb_size < bb_size or max_index is None:
                max_index = idx

        if max_index is not None:
            traffic_light_id = results[max_index]['id']

        return id_mapping[traffic_light_id]


class TestTLClassifier(object):

    def __init__(self):
        self.detector = TLClassifier()

    def test_classification(self):
        # Load image
        image_path_green = ('src/tl_detector/light_classification/test_images/green.jpg', TrafficLight.GREEN)
        image_path_yellow = ('src/tl_detector/light_classification/test_images/yellow.jpg', TrafficLight.YELLOW)
        image_path_red = ('src/tl_detector/light_classification/test_images/red.jpg', TrafficLight.RED)
        image_path_na = ('src/tl_detector/light_classification/test_images/NA.jpg', TrafficLight.UNKNOWN)

        for image_path in [image_path_green, image_path_yellow, image_path_red, image_path_na]:
            image = np.asarray(Image.open(image_path[0]))
            image = np.expand_dims(image, 0)
            gt_result = image_path[1]
            pred_result = self.detector.get_classification(image)
            print(image_path[0])
            print('Prediction success: ' + str(gt_result == pred_result))

            if gt_result != pred_result:
                raise Exception('Prediction error.')

    def measure_time(self):
        # Load image
        image_path = 'src/tl_detector/light_classification/test_images/green.jpg'
        image = np.asarray(Image.open(image_path))
        image = np.expand_dims(image, 0)

        repeats = 25

        t0 = datetime.datetime.now()
        for i in range(repeats):
            _ = self.detector.get_classification(image)

        delta = datetime.datetime.now() - t0
        print('Time per image in ms: ' + str(delta.seconds * 100.0 / float(repeats)))


if True:
    tester = TestTLClassifier()
    tester.measure_time()
    tester.test_classification()
