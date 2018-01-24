import os
import PIL
import rospy
import tensorflow as tf
import numpy as np
from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        cwd = os.getcwd()

        self.clsmap = {
            1 : TrafficLight.GREEN,
            2 : TrafficLight.RED,
            3 : TrafficLight.YELLOW,
            4 : TrafficLight.UNKNOWN
        }
        self.clasname = {
            1 : "GREEN",
            2 : "RED",
            3 : "YELLOW",
            4 : "UNK",
        }
        #TODO load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(os.getcwd() + '/frozen_inference_graph.pb', 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.session = tf.Session(graph=self.detection_graph)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
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
        #img_pil = PIL.Image.fromarray(image)
        #img_pil.save("/home/shangliy/test.jpg")
        image_exp = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num) = self.session.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_exp})
        classes = np.squeeze(classes)
        scores = np.squeeze(scores)
        ind = np.argmax(scores)

        if scores[ind] >= 0.5:
            rospy.loginfo("----Light status = %s"%(self.clasname[int(classes[ind])]))
        
        return self.clsmap[int(classes[ind])]
