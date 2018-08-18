from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import tensorflow as tf
import rospy
import traceback
import json
import time

GRAPH_FILE = '/home/gabymoynahan/CarND-Capstone/data/graphs/frozen_inference_graph.pb'
#GRAPH_FILE = '/home/gabymoynahan/CarND-Capstone/data/graphs/ssd_inception_v2_coco_11_06_2017/frozen_inference_graph.pb'
#GRAPH_FILE = '/home/gabymoynahan/CarND-Capstone/data/graphs/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'

METHOD = 0 #0 = Image Processing Only, 1 = Image Processing & SSD
BOXES_SCORE_MIN = 0.5 #Minimum passing score for detection

'''
def log(arg):
	#writes log information to debug file
	with open('logfile.txt','a') as f:
		f.write(arg+'\n')
'''

def load_graph(graph_file):
#Loads a frozen inference graph
	graph = tf.Graph()
	with graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(graph_file, 'rb') as fid:
			serialized_graph = fid.read()
			od_graph_def.ParseFromString(serialized_graph)
			tf.import_graph_def(od_graph_def, name='')
	return graph

#----------FUNCTIONS FOR CLASSIFICATION----------

def draw_boxes(img, bboxes, color=(0, 0, 255), thick=3):
	# Draws bounding boxes
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
	#print ("length of boxes", n)
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
		trimmed_image = image[ y:y+h , x:x+w ]

		#return trimmed_image
		#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/trimmed_{}.png'.format(time.time()),trimmed_image)
		images.append(trimmed_image)

	return images

def TLImage_Pro(image):
	# Image processing using openCV
	image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/HSV_{}.png'.format(time.time()),image_hsv)
	lower_red = np.array([0,50,50])
	upper_red = np.array([10,255,255])
	red1 = cv2.inRange(image_hsv, lower_red , upper_red)

	lower_red = np.array([170,50,50])
	upper_red = np.array([180,255,255])
	red2 = cv2.inRange(image_hsv, lower_red , upper_red)
	converted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)
	#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/converted_{}.png'.format(time.time()),converted_img)
	blur_img = cv2.GaussianBlur(converted_img,(15,15),0)

	circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=120)
	#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/circles_{}.png'.format(time.time()),circles)
	return circles

'''
with open('logfile.txt','wb') as f:
    #creates logfile from scratch
    f.write('new log file \n')
'''
#log('log file initialized')
#log('tf version: {}'.format(tf.__version__))


detection_graph = load_graph(GRAPH_FILE)

image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
#log ('using pretrained model: {}'.format(GRAPH_FILE))

sess = tf.Session(graph = detection_graph)
#log('tf session open')


class TLClassifier(object):
	def __init__(self):
		#TODO load classifier
		print ("Classifier: initialized")


	def get_classification(self, image):
		#Preprocess camera image

		#Image Processing Only
		if METHOD == 0:
			img_circles = TLImage_Pro(image)
			if img_circles is not None:
				return TrafficLight.RED

			else:
				return TrafficLight.UNKNOWN



		#SSD with Image Processing
		elif METHOD == 1:

			#Blur the image so it seems more realistic and the classificator performs better
			gbeq_image = cv2.GaussianBlur(image, (5,5),0)
			boxes, scores, classes = TLDetection(gbeq_image, sess)
			#log('CAMERA IMAGE PROCESSED, {} boxes detected'.format(len(boxes)))
			boxes, scores, classes = TLBoxes(BOXES_SCORE_MIN, boxes, scores, classes)

			image_height = image.shape[0]
			image_width = image.shape[1]
			box_coordinates = TLResizeBoxes(boxes, image_height, image_width)
			#trimmed_lights = TLTrim(image, box_coordinates)


			if len(boxes) != 0:
				print ("found boxes with prob > 0.5: ", boxes)

				img_circles = TLImage_Pro(gbeq_image)
				processed_image = draw_boxes(gbeq_image, box_coordinates, color=(0, 0, 255), thick=3)
				#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/file_{}.png'.format(time.time()),processed_image)
				#log('processed camera image saved {}'.format(time.time()))
				#cv2.imwrite('/home/gabymoynahan/CarND-Capstone/data/processed_images/circles_{}.png'.format(time.time()),img_circles)

				if img_circles is not None:
					print ("Red!")
					return TrafficLight.RED

				else:
					print ("NOT red")
					return TrafficLight.UNKNOWN
