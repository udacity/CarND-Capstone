import time
import tensorflow as tf
import numpy as np

from object_detection.utils import visualization_utils as vis_util
from object_detection.utils import label_map_util

class TLClassifier(object):

    def __init__(self, path_to_ckpt, label_path, num_classes):
        self.detection_graph = tf.Graph()
        self.session = tf.Session(graph=self.detection_graph)
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                label_map = label_map_util.load_labelmap(label_path)
                categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
                self.category_index = label_map_util.create_category_index(categories)

    # def load_image_into_numpy_array(self, image):
    #     (im_width, im_height) = image.size
    #     return np.array(image.getdata()).reshape(
    #         (im_height, im_width, 3)).astype(np.uint8)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        start = time.time()
        #image_np = self.load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num) = self.session.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        # vis_util.visualize_boxes_and_labels_on_image_array(
        #   image_np,
        #   np.squeeze(boxes),
        #   np.squeeze(classes).astype(np.int32),
        #   np.squeeze(scores),
        #   self.category_index,
        #   use_normalized_coordinates=True,
        #   line_thickness=8,min_score_thresh=0.5)

        final_classes = [c for c, s in zip(classes[0], scores[0]) if s > 0.5]
        return image_np_expanded, final_classes, time.time() - start
