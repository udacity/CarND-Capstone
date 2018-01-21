import os
import cv2
import time
import rospy
import numpy as np
import tensorflow as tf

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from styx_msgs.msg import TrafficLight
from cv_bridge import CvBridge, CvBridgeError


class Timer:
    def __init__(self, message=''):
        self.message = message

    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        message = '{} in {} seconds'.format(self.message, time.clock() - self.start)
        rospy.loginfo(message)


# Function to load a graph from a proto buf file.
def _load_graph(graph_file, config, verbose=False):
    with tf.Session(graph = tf.Graph(), config=config) as sess:
        assert tf.get_default_session() is sess

        gd = tf.GraphDef()
        with tf.gfile.Open(graph_file, 'rb') as f:
            data = f.read()
            gd.ParseFromString(data)

        tf.import_graph_def(gd, name='')
        graph = tf.get_default_graph()
        if verbose:
            print ('Graph v' + str(graph.version) + ', nodes: ' + ', '
                   .join([n.name for n in graph.as_graph_def().node]))

        return graph


# Pull out the traffic light box with the highest score of approach.
def _extract_box(boxes, scores, classes, confidence, img_width, img_height):
    # Preparation of values.
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    scores = np.squeeze(scores)

    # Get the box with the highest score.
    max_conf = 0
    number = -1
    for i in range(boxes.shape[0]):
        if scores[i] > confidence and classes[i] == 10:
            if scores[i] > max_conf:
                max_conf = scores[i]
                number = i

    if number != -1:
        # Create a tuple for each box.
        box = tuple(boxes[number].tolist())

        # The corners of the box are obtained.
        min_y, min_x, max_y, max_x = box
        left, right, top, bottom = (min_x * img_width, max_x * img_width,
                                    min_y * img_height, max_y * img_height)

        # They expand a little more.
        left -= 5
        if left < 0:
            left = 0

        top -= 10
        if top < 0:
            top = 0

        bottom += 10
        if bottom > img_height:
            bottom = img_height

        right = right + 5
        if right > img_width:
            right = img_width

        box = int(left), int(right), int(top), int(bottom)
        return box
    else:
        return None


class TLClassifier(object):
    def __init__(self, model_det_dir = None):
        # Load Classifier.

        # Get model directory and check model files.
        if model_det_dir is None:
            import rospkg
            rp = rospkg.RosPack()
            model_det_dir = os.path.join(rp.get_path('tl_detector'), 'model')

        rospy.loginfo('Using model directory {}'.format(model_det_dir))

        model_det_path = os.path.join(model_det_dir, 'model_detection.pb')
        if not os.path.exists(model_det_path):
            rospy.logerr('Detection model not found at {}'.format(model_det_path))

        model_clas_path = os.path.join(model_det_dir, 'model_classification.pb')
        if not os.path.exists(model_clas_path):
            rospy.logerr('Classification model not found at {}'.format(model_clas_path))

            # Activate optimizations for Tensorflow.
            # log_device_placement = True
            self.config = tf.ConfigProto(device_count={'GPU': 1, 'CPU': 1})
            self.config.gpu_options.allow_growth = True
            self.config.gpu_options.per_process_gpu_memory_fraction = 1
        else:
            self.config = tf.ConfigProto()

        # Optimizer
        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level

        # Load graphs.
        self.graph_detection = _load_graph(model_det_path, self.config)
        self.graph_classification = _load_graph(model_clas_path, self.config)

        # Create the tensorflow sessions.
        self.sess_detection = tf.Session(graph=self.graph_detection, config=self.config)
        self.sess_classification = tf.Session(graph=self.graph_classification, config=self.config)

        # Definition of input and output tensors for the detection graph.
        self.image_tensor = self.graph_detection.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image in which a specific object has been located.
        self.detection_boxes = self.graph_detection.get_tensor_by_name('detection_boxes:0')

        # For each detected object the score represents the confidence level.
        self.detection_scores = self.graph_detection.get_tensor_by_name('detection_scores:0')

        # This is the MS COCO dataset class, we only need class 10 for the traffic light.
        self.detection_classes = self.graph_detection.get_tensor_by_name('detection_classes:0')

        # Input and output tensors are obtained from the classification.
        self.in_graph = self.graph_classification.get_tensor_by_name('input_1_1:0')
        self.out_graph = self.graph_classification.get_tensor_by_name('output_0:0')

        # Index of models for the traffic light detection message.
        self.index2msg = {0: TrafficLight.RED, 1: TrafficLight.GREEN, 2: TrafficLight.YELLOW}
        self.index2color = {0: (255, 0, 0), 1: (0, 255, 0), 2: (255, 255, 0)}

        # Traffic light publisher.
        self.bridge = CvBridge()
        self.traffic_light_pub = rospy.Publisher('/tld/traffic_light', Image, queue_size=1)

        # Quick classification for preload models.
        self.detection(cv2.cvtColor(np.zeros((600, 800), np.uint8), cv2.COLOR_GRAY2RGB))
        self.classification(cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        
        Args:
            image (cv::Mat): image containing the traffic light
        
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Light color prediction.

        with Timer('get_classification'):
            img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            box = self.detection(img)
            if box is None:
                if rospy.get_param('~publish_traffic_light', False):
                    self.traffic_light_pub.publish(
                        self.bridge.cv2_to_imgmsg(
                            cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB), "rgb8"
                        )
                    )
                return TrafficLight.UNKNOWN

            left, right, top, bottom = box
            img_crop = img[top:bottom, left:right]
            traf_light = cv2.resize(img_crop, (32, 32))

            return self.classification(traf_light)

    def detection(self, img):
        # Traffic light detection.

        img_height, img_width, _ = img.shape
        img_expanded = np.expand_dims(img, axis=0)

        with self.sess_detection.as_default(), self.graph_detection.as_default(), Timer('detection'):
            boxes, scores, classes = self.sess_detection.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: img_expanded}
            )

            # Extract box and append to list.
            return _extract_box(boxes, scores, classes, 0.1, img_width, img_height)

    def classification(self, img):
        # Traffic light classification.

        with self.sess_classification.as_default(), self.graph_classification.as_default(),\
             Timer('classification'):

            probs = self.sess_classification.run(
                tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [img]}))
            )
            sfmax = list(probs)
            predicted_idx = sfmax.index(max(sfmax))
            # predicted_idx = np.argmax(probs)

            # Add a colored bbox and publish traffic light if needed
            # -> rosparam set /tl_detector/publish_traffic_light true.
            if rospy.get_param('~publish_traffic_light', False):
                cv2.rectangle(img, (0, 0), (31, 31), self.index2color[predicted_idx], 1)
                self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

        return self.index2msg[predicted_idx]


if __name__ == "__main__":
    classifier = TLClassifier(model_dir='model')
    classifier.publish_traffic_light = False

    img_dir = 'images'

    # For local tests.
    for t_light_name in ['Red_Light_1.png', 'Off_1.png', 'Green_Light_1.png', 'Yellow_Light_1.png']:
        img = cv2.imread(os.path.join(img_dir, t_light_name))
        color = classifier.get_classification(img)
        print (color)
