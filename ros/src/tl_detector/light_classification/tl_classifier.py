from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
import numpy as np
import time
from PIL import Image


DETECTION_THRESHOLD = 0.5

class TLClassifier(object):
    def __init__(self, model):
	self.state = TrafficLight.UNKNOWN

	# Get model name according to simulator or site launch files definition
        TRAFFIC_CLASSIFIER_MDOEL_PATH = model
        #TODO load classifier
	self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(TRAFFIC_CLASSIFIER_MDOEL_PATH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

	    self.sess = tf.Session(graph=self.detection_graph)

	self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
    	self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        """
	with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                self.sess = sess
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        """

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

	if 'session' in locals() and session is not None:
    	    print('Close interactive session')
            session.close()

        time_start = time.time()
        #TODO implement light color prediction
        #image_np = self.__preprocess_image(image)
       	image_np = image 
        
       	# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        time0 = time.time()

        # Actual detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})

        time1 = time.time()

        output = self.__postprocessing_detected_box(scores[0], classes[0])
        rospy.loginfo('Time in seconds' + str(time1-time_start)+' Result:'+self.__traffic_id_to_name(output))
        return output


    def __preprocess_image(self, image):
        image = Image.fromarray(image)
        # image.save('check'+str(time.time())+'.jpg')
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def __postprocessing_detected_box(self, scores, classes):
        candidate_num = 4
        vote = []
        for i in range(candidate_num):
            if scores[i] < DETECTION_THRESHOLD:
                break
            vote.append(self.__label_map_to_traffic_light(int(classes[i])))
        #rospy.logerr('Votes' + str(vote))
        if vote:
            return max(vote, key=vote.count)
        else:
            return 4

    def __label_map_to_traffic_light(self, label_id):
        traffic_label = int(label_id) - 1
        if traffic_label in [0, 1, 2, 4]:
            return  traffic_label
        return 4

    def __traffic_id_to_name(self, traffic_id):
        traffic_light_names = ['Red','Yellow','Green','Error','Unknown']
        return traffic_light_names[traffic_id]

