from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self, is_sim):
        
        #Depending if we have the simulation or carla choose the correct model
        
        if is_sim:
            PATH_TO_GRAPH = r'light_classification/traffic_models/sim_v4/frozen_inference_graph.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/traffic_models/carla_v4/frozen_inference_graph.pb'
        
        #Load frozen graph -- TENSORFLOW API OBJECT DETECTION -- MOBILENET MODEL USED
        self.graph = tf.Graph()
        
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')
        
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
    
        self.sess = tf.Session(graph=self.graph)
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
            
            Args:
            image (cv::Mat): image containing the traffic light
            
            Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            
            """
        #Find traffic light in the given image
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                                            [self.boxes, self.scores, self.classes, self.num_detections],
                                            feed_dict={self.image_tensor: img_expand})
        
        #Remove single-dimensional entries
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        
        #Return the int of the traffic light if detected else UNKNOWN
        if classes[0] == 1:
            return TrafficLight.GREEN
        elif classes[0] == 2:
            return TrafficLight.RED
        elif classes[0] == 3:
            return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
