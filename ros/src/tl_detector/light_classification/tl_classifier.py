from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy
import yaml
import cv2
from PIL import Image as PIL_Image

class TLClassifier(object):
    def __init__(self):

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.graph = tf.Graph()
        self.threshold = .45
        PATH_TO_GRAPH = r"{}".format(self.config["model"])
        #The model is not sensitive to yellow lights, i.e, may classify a yellow light as green.
        #Added this "feature engineering" to be conservative .
        self.force_to_yellow = self.config["force_to_yellow"]



        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            try: 
                with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                    od_graph_def.ParseFromString(fid.read())
                    tf.import_graph_def(od_graph_def, name='')
                    rospy.loginfo("used light classifier model:{}".format(PATH_TO_GRAPH))
            except EnvironmentError:
                rospy.logfatal("Can't load model:".format(PATH_TO_GRAPH))

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image,true_state):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        def saveImage(img):
            #img = cv2.resize(img, (224,224))
            img= PIL_Image.fromarray(img, 'RGB')
            img.save("test.png", "PNG")

        #h, w = image.shape[:2]
        #image = cv2.resize(image, (h//2,w//2))
        #saveImage(image)
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            #rospy.logdebug(c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        
        #rospy.logdebug ('SCORE: {} Class: {}'.format(scores[0],classes[0]))
        state = TrafficLight.UNKNOWN
        if scores[0] > self.threshold:
            if classes[0] == 1:
                state = TrafficLight.GREEN
            elif classes[0] == 2:
                state = TrafficLight.RED
            elif classes[0] == 3:
                state = TrafficLight.YELLOW
            if (self.force_to_yellow):
                h, w = image.shape[:2]
                crop = map(int, (boxes[0][0]*h , boxes[0][2]*h, boxes[0][1]*w,  boxes[0][3]*w))
                #saveImage(image[crop[0]:crop[1], crop[2]:crop[3]])
                cropped = image[crop[0]:crop[1], crop[2]:crop[3]]
                hist_b = np.squeeze(cv2.calcHist([cropped],[0],None,[8],[0,256]))
                hist_g = np.squeeze(cv2.calcHist([cropped],[1],None,[8],[0,256]))
                hist_r = np.squeeze(cv2.calcHist([cropped],[2],None,[8],[0,256]))
                b = hist_b[-1]
                g = hist_g[-1]
                r = hist_r[-1]
                avg = np.mean((b, g, r))
                if  state != TrafficLight.RED and r>0 and 1.1 > g/r > 0.9 and b/r < 0.1:
                    state = TrafficLight.YELLOW
                    #print ("forced state light to Yellow")

            #print(">>>>>>>>>>>>")
            #print("true:", true_state, "detected:", state, "score:", scores[0])
            #print(map(int, hist_b))
            #print(map(int, hist_g))
            #print(map(int, hist_r))
            #print("<<<<<<<<<<<")

            #rospy.loginfo ('Detected. SCORE: {} Class: {} box:{}'.format(scores[0],classes[0], boxes[0]))
        #else:
            #rospy.loginfo ('Undetected. SCORE: {} Class: {} box:{}'.format(scores[0],classes[0], boxes[0]))
        return state
