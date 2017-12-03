from styx_msgs.msg import TrafficLight
import rospy
import yaml
import numpy as np
import tensorflow as tf
from .utils import label_map_util

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        PATH_TO_CKPT = 'light_classification/{}/frozen_inference_graph.pb'.format(self.config['model'])
        PATH_TO_LABELS = 'light_classification/udacity_labels.pbtxt'
        NUM_CLASSES = 3

        #load frozen graph
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        #load labels
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)

        with detection_graph.as_default():
          with tf.Session(graph=detection_graph) as self.sess:
            # Definite input and output Tensors for detection_graph
            self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        #image_np = self.load_image_into_numpy_array(image)
        image_np = image
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        top_3_classes = list(classes[0][:3])
        top_predicted_class_int = max(set(top_3_classes), key=top_3_classes.count)

        #int -> class mapping
        if top_predicted_class_int == 1:
            return TrafficLight.GREEN
        elif top_predicted_class_int == 2:
            return TrafficLight.RED
        elif top_predicted_class_int == 7:
            return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
