from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import tensorflow as tf
import rospy
import traceback
import json
import time

# Frozen inference graph files. NOTE: change the path to where you saved the models.
#------------------------------------------------------------------------------------
#SSD MOBILENET V1 on COCO  ---> Good performance using CPU, not best results but enough for simulator 

GRAPH_FILE = '/home/student/catkin_ws/CarND-Capstone/data/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'

#FASTER RCNN RESNET101 on COCO ---> VERY Poor performance on CPU, but QUITE GOOD resutls in classification, modify score to 0.75 or more

#GRAPH_FILE = '/home/student/catkin_ws/CarND-Capstone/data/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb'

#SSD INCEPTION V2 on COCO ---> Poor performance sometimes, penalizes results, but QUITE GOOD classification.

#GRAPH_FILE = '/home/student/catkin_ws/CarND-Capstone/data/ssd_inception_v2_coco_2017_11_17/frozen_inference_graph.pb'
#-------------------------------------------------------------------------------------

DEBUG_MODE = 1 #debug mode activates the classifier to save processed images with boxes. Use 0 for production
BOXES_SCORE_MIN = 0.5 #minimum score to accept box detection

#Useful functions to load and record graph

with open('logfile.txt','wb') as f:
    #creates logfile from scratch
    f.write('new log file \n')

def log(arg):
    #writes log information to debug file
    with open('logfile.txt','a') as f:
        f.write(arg+'\n')
        
def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

log('log file initialized')
log('tf version: {}'.format(tf.__version__))

detection_graph = load_graph(GRAPH_FILE)

#log('loaded graph with following nodes:')
#for op in tf.get_default_graph().get_operations():
#	log('---{}'.format(op.name))

image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
log ('using pretrained model: {}'.format(GRAPH_FILE))

sess = tf.Session(graph = detection_graph)
log('tf session open')


#Useful functions for classification

def equalize_image(image):
    eq_image = np.copy(image)
    for c in range(0,2):
        eq_image[:,:,c] = cv2.equalizeHist(image[:,:,c])
    cv2.imwrite('eq_sample.png',eq_image)
    return eq_image

def gaussBlur_image(image):
    gb_image = cv2.GaussianBlur(image, (5,5),0)
    cv2.imwrite('gb_sample.png',gb_image)
    return gb_image


def draw_boxes(img, bboxes, color=(0, 0, 255), thick=3):
	#draws bounding boxes
	# Make a copy of the image
	imcopy = np.copy(img)
	# Iterate through the bounding boxes
	for bbox in bboxes:
		# Draw a rectangle given bbox coordinates
		cv2.rectangle(imcopy, (bbox[1], bbox[0]),(bbox[3], bbox[2]), color, thick)
	# Return the image copy with boxes drawn
	return imcopy

def TLDetection(image, sess):

	image_np = np.expand_dims(np.asarray(image, dtype = np.uint8), 0)
	boxes, scores, classes = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict = {image_tensor: image_np})

	boxes = np.squeeze(boxes)
	scores = np.squeeze(scores)
	classes = np.squeeze(classes)

	return boxes, scores, classes

def TLBoxes(prob, boxes, scores, classes):
	#filter boxes under minimum probability 'prob'
	#COCO class index for TrafficLight is '10'
	
	n = len(boxes)
	idxs = []
	for i in range(n):
		if scores[i] >= prob and classes[i] == 10:
			idxs.append(i)

	filtered_boxes = boxes[idxs, ...]
	filtered_scores = scores[idxs, ...]
	filtered_classes = classes[idxs, ...]
	return filtered_boxes, filtered_scores, filtered_classes

def TLResizeBoxes(boxes, image_height, image_width):
	#Resize boxes from original values (0:1) to image size
	box_coords = np.zeros_like(boxes)
	box_coords[:,0] = boxes[:,0] * image_height
	box_coords[:,1] = boxes[:,1] * image_width
	box_coords[:,2] = boxes[:,2] * image_height
	box_coords[:,3] = boxes[:,3] * image_width

	return box_coords

def TLTrim(image, box_coordinates):
	#return trimmed images containing all traffic light signals ahead: from
	#zero (no traffic lights) to whatever
	
	images = []
	for box in box_coordinates:
		x = int(np.round(box[1]))
		y = int(np.round(box[0]))
		w = int(np.round(box[3] - box[1]))
		h = int(np.round(box[2] - box[0]))
		images.append(image[ y:y+h , x:x+w ])
	return images

def TLGetColor(image):
	#check a trimmed image containing only one traffic light (red+yellow+green bulbs)
	#and returns the color that is ON
	
        states = [TrafficLight.GREEN, TrafficLight.YELLOW, TrafficLight.RED]

        best_state = TrafficLight.UNKNOWN
        for state in states:

            _image = image.copy()

            #let's pick some 6x6 pixels in the middle of each light bulb:
            #      
            #      x coord, we cut in the middle *0.5
            #      |
            #   -------
            #   |     |
            #   |     |- y coord, first third of light, we cut at 0.2
            #   |-----|
            #   |     |_ y coord, second third of light, we cut at 0.55
            #   |     |                                                            
            #   |-----|
            #   |     |_ y coord, third part of light, we cut at 0.85
            #   |     |
            #   -------            
            
                
            if state == TrafficLight.RED:
                _image = _image[int(image.shape[0] * 0.2) - 3: int(image.shape[0] * 0.2) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]
            if state == TrafficLight.YELLOW:
                _image = _image[int(image.shape[0] * 0.55) - 3: int(image.shape[0] * 0.55) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]
            if state == TrafficLight.GREEN:
                _image = _image[int(image.shape[0] * 0.85) - 3: int(image.shape[0] * 0.85) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]
            
            # So we pick the bulb that is on...
            if _image[np.where(np.squeeze(_image) > 250)].shape[0] > 15:
                best_state = state

        return best_state


class TLClassifier(object):
    def __init__(self):
	log('Classifier: initialized')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction

        #preprocess camera image, Blur the image so it seems more realistic and the classificator performs better
        #image = equalize_image(image)
        gbeq_image = gaussBlur_image(image)

        boxes, scores, classes = TLDetection(gbeq_image, sess)
        log('CAMERA IMAGE PROCESSED, {} boxes detected'.format(len(boxes)))
        boxes, scores, classes = TLBoxes(BOXES_SCORE_MIN, boxes, scores, classes)
        image_height = image.shape[0]
        image_width = image.shape[1]
        box_coordinates = TLResizeBoxes(boxes, image_height, image_width)
        trimmed_lights = TLTrim(image, box_coordinates)
	
        #if debud mode is active, then saves processed images with boxes
        if DEBUG_MODE == 1:
	    processed_image = draw_boxes(image, box_coordinates, color=(0, 0, 255), thick=3)
	    cv2.imwrite('img_processed/file_{}.png'.format(time.time()),processed_image)
            #log('processed camera image saved {}'.format(time.time()))
	
        lights = []
        for light_bulbs in trimmed_lights:
            color = TLGetColor(light_bulbs)
	    lights.append(color)
	log('lights detected: {}'.format(lights))
	
        for light_bulb in lights:
            if light_bulb in [TrafficLight.RED]: # check if RED is detected in any light
                return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN

