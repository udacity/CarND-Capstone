#from styx_msgs.msg import TrafficLight
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor
import time
from scipy.stats import norm
import cv2
import tensorflow as tf
import rospy

GRAPH_FILE = 'ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self):

        self.traffic_light_list = []
        self.graph_file = GRAPH_FILE
        
        cmap = ImageColor.colormap
        #print("Number of colors =", len(cmap))
        self.COLOR_LIST = sorted([c for c in cmap.keys()])

        #TODO load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.frame_count = 0

    
    def detect_traffic_lights(self, img, confidence_level=0.2, detect_class_id=[10]):
        #rospy.loginfo("Detection start: %s", time.time())
        image = Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))  
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
        width, height = image.size

        with tf.Session(graph=self.detection_graph) as sess:                
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: image_np})

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = self.filter_boxes(confidence_level, boxes, scores, classes, detect_class_id)

        box_coords = self.to_image_coords(boxes, height, width)
        self.traffic_light_list = []
        #print(box_coords)
        for box in box_coords:
            top,left,bottom,right = box
            traffic_light = image.crop(
                (
                left,
                    top,
                    right,
                    bottom
                )
            )
            self.traffic_light_list.append(traffic_light)
        
        self.draw_boxes(image, box_coords, classes)
        cv2_output_img = cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)  
        
        #cv2.imshow("Image window", cv2_output_img)
        #cv2.waitKey(3)

        #pic_filename = "./result/%08d.png"%self.frame_count
        #image.save(pic_filename)
        #self.frame_count += 1
        if len(scores) < 1:
            rospy.loginfo("No traffic light!")
        else:
            rospy.loginfo("Traffic light ahead!")
        for i in range(min(3,len(scores))):
            rospy.loginfo("Top prob #%d: %.4f", i, scores[i])

        return cv2_output_img
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
        pass


    
    def filter_boxes(self, min_score, boxes, scores, classes, filter_classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if classes[i] in filter_classes and scores[i] >= min_score:
                idxs.append(i)
        
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes


    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        
        return box_coords

    def draw_boxes(self, image, boxes, classes, thickness=4):
        """Draw bounding boxes on the image"""
        draw = ImageDraw.Draw(image)
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            class_id = int(classes[i])
            color = self.COLOR_LIST[class_id]
            draw.line([(left, top), (left, bot), (right, bot), (right, top), (left, top)], width=thickness, fill=color)
            