from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        
        MODEL_PATH = '/floyd/home/TL/trained_models/ssdlite_mobilenet_v2_coco_2018_05_09_exported/frozen_inference_graph.pb'
        
        # Load the graph
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(MODEL_PATH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        
        self.detection_graph.as_default()
        self.sess = tf.Session(graph=self.detection_graph)
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            
    
    def get_classification(self, image):
        classes, scores, boxes = self.detect(image)
        if (1 in classes):
            return TrafficLight.RED
            # return "Red"
        elif(2 in classes):
            return TrafficLight.YELLOW
            # return "Yellow"
        elif(3 in classes):
            return TrafficLight.GREEN
            # return "Green"
        else:
            return TrafficLight.UNKNOWN
            # return "Unknown"
        
                
    def detect(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image_np_expanded = np.expand_dims(image, axis=0)
        
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
          [self.detection_boxes, 
           self.detection_scores, 
           self.detection_classes, 
           self.num_detections],
          feed_dict={self.image_tensor: image_np_expanded})
        
        final_classes = []
        final_scores = []
        final_boxes = []
        num_detections = int(num[0])
#         print('num_detections:', num_detections)
#         print('scores:', scores)
        for i in range(num_detections):
            classId = classes[0][i]
            score = scores[0][i]
            box = boxes[0][i]
            if score > 0.5:
                final_scores.append(score)
                final_classes.append(classId)
                final_boxes.append(box)
        return final_classes, final_scores, final_boxes
    



    def close(self):
        self.sess.close()
