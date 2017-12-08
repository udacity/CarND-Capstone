import os
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
from functools import partial

def joinfiles(directory, filename):
    """
    directory is the chunks directory
    filename is the name of assembled file.
    """
    chunksize = 1024
    maxchunks = 1024 * 5
    rospy.loginfo("restoring:" + filename +" from directory:" + directory)
    if os.path.exists(directory):
        if os.path.exists(filename):
            os.remove(filename)
        # end of if os.path.exists(filename)
        output = open(filename, 'wb')
        chunks = os.listdir(directory)
        chunks.sort()
        for fname in chunks:
            rospy.loginfo("Joining " + fname + " out of " + str(len(chunks)))
            fpath = os.path.join(directory, fname)
            with open(fpath, 'rb') as fileobj:
                for chunk in iter(partial(fileobj.read, chunksize * maxchunks), ''):
                    output.write(chunk)
                # end of for chunk in iter(partial(fileobj.read, chunksize * maxchunks), '')
            # with open(fpath, 'rb') as fileobj
        # end of for fname in chunks
        output.close()
    # end of os.path.exists(directory)

class TLClassifier(object):
    def __init__(self, sim):
        #DONE load classifier
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        if sim:
            model_folder = '/sim_model'
        else:
            model_folder = '/real_model'
        # end of if sim
    
        model_path = curr_dir + model_folder + '/frozen_inference_graph.pb'
        chunk_folder = curr_dir + model_folder + '/chunks'
        if not os.path.exists(model_path):
            joinfiles(chunk_folder, model_path)
    
        self.detection_graph = tf.Graph()
    
        with self.detection_graph.as_default():
    
            od_graph_def = tf.GraphDef()
    
            with tf.gfile.GFile(model_path, 'rb') as fid:
    
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            # end of with tf.gfile.GFile(model_path, 'rb') as fid:
        # with self.detection_graph.as_default():
        self.session = tf.Session(graph=self.detection_graph)
    
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def convert_class_to_color(self, class_value):
        if class_value == 1:
            light_color = TrafficLight.GREEN
            color_label = "GREEN"
        elif class_value == 2:
            light_color = TrafficLight.RED
            color_label = "RED"
        elif class_value == 3:
            light_color = TrafficLight.YELLOW
            color_label = "YELLOW"
        else:
            light_color = TrafficLight.UNKNOWN
            color_label = "UNKNOWN"
        # end of if class_value == 1
        return (light_color, color_label)
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
          Args:
              image (cv::Mat): image containing the traffic light
          Returns:
              int: ID of traffic light color (specified in styx_msgs/TrafficLight)
          """
        #DONE implement light color prediction
        image_expanded = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num_det) = self.session.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})
    
        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)
    
        # Print class based on best score
    
        light_color = TrafficLight.UNKNOWN
        color_label = "UNKNOWN"
    
        # find index with the max score[index]
        max_score = scores[0]
        max_index = 0
        for i in range(1, boxes.shape[0]):
            if max_score < scores[i]:
                max_score = scores[i]
                max_index = i
            # end of if max_score < scores[i]
        # end of for i in range(1, boxes.shape[0])
    
        light_color, color_label = self.convert_class_to_color(classes[max_index])
        if max_score < 0.5: # used to be 0.7
            light_color, color_label = TrafficLight.UNKNOWN, "UNKNOWN"
        # end of if max_score < 0.7
        # rospy.loginfo("Traffic Light Color value: %r, label: %s; score: %f" %
        #               (light_color, color_label, max_score))
        return light_color
