#!/usr/bin/env python
from net.build import TFNet
import sys
import argparse
import threading
import numpy as np
import cv2
import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from light_classification.tl_classifier import TLClassifier
import tensorflow as tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.model = self.get_model_callback()
        self.predict = self.process
        self.bridge = CvBridge()
        self.boxes = None
        self.image_counter = 0
        self.img = None
        self.img_out = None
        self.image_lock = threading.RLock()
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	rospy.loginfo("tl_detector.py init()")
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        #rospy.Timer(rospy.Duration(0.04),self.callbackImage)
	rospy.loginfo("tl_detector.py init()->rospy.spin()")
        rospy.spin()


    def process(self,model,img):
	#rospy.loginfo("tl_detector.py process()")
	result = model.return_predict(img[None,:,:,:])
	return result


    def pose_cb(self, msg):
	#rospy.loginfo("tl_detector.py pose_cb()")
        self.pose = msg

    def waypoints_cb(self, waypoints):
	#rospy.loginfo("tl_detector.py waypoints_cb()")
        self.waypoints = waypoints

    def traffic_cb(self, msg):
	#rospy.loginfo("tl_detector.py traffic_cb()")
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
	#rospy.loginfo("tl_detector.py image_cb() received image by message")
        self.has_image = True
        self.image_counter = self.image_counter +1
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
	    #rospy.loginfo("tl_detector.py image_cb() self.state != state")
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
	    #rospy.loginfo("tl_detector.py image_cb() self.state_count >= THRESHOLD")
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
	    #rospy.loginfo("tl_detector.py image_cb() publish RED LIGHT waypoint")
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
	    #rospy.loginfo("tl_detector.py image_cb() publish RED LIGHT last_waypoint")
        self.state_count += 1



    def find_next_stop_line(self,stop_line_positions):
        """
        Args: car position , list of stop_line positions
        Returns:
            int: index of the closest stop line in stop_line_positions 

        """
	#rospy.loginfo("find_next_stop_line()")
        car_position = [self.pose.pose.position.x, self.pose.pose.position.y]
	#rospy.loginfo("car_position  %s ",car_position)   # [1148.56 1184.65]


        min_dist = 999999
        min_ind = 0
        ind = 0

        # Calcuate distance between car and waypoint
        for stop_line in stop_line_positions:
	    #rospy.loginfo("traffic_light_waypoint  %s ",stop_line)   # [1148.56 1184.65]
            dist = self.get_euclidean_distance(car_position, stop_line)
            
            # Store the index of waypoint which is nearest to the car
            if dist < min_dist:
                min_dist = dist
                min_ind = ind
        
	next_stop_line = stop_line_positions[ind]
        return next_stop_line




    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	#rospy.loginfo("get_light_state()")
        if(not self.has_image):
            self.prev_light_loc = None
	    rospy.loginfo("tl_detector.py get_light_state()-> No Image received cannot classify: returning")
            return False
        if(np.remainder(self.image_counter,20 )):
            self.boxes = None
            self.img_out = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            return self.light_classifier.get_classification(self.img_out,self.boxes)
	else:
            rospy.loginfo("processing image %d ",self.image_counter)

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        height, width ,channels = cv_image.shape   # 600 800 3
        if self.image_lock.acquire(True):
            self.img = cv_image
            if self.model is None:
                self.model = self.get_model_callback()
            rospy.loginfo(" get_light_state() predict image size h: %d  w: %d c: %d ",height,width,channels)
            self.img_out, self.boxes = self.predict(self.model,self.img)
            self.image_lock.release()
            rospy.loginfo(" get_light_state() received prediction")
            self.img_out = np.asarray(self.img_out[0,:,:,:])
            for box in self.boxes:
                if 'traffic light' in box['label']:
                    cv2.rectangle(self.img_out,(box['topleft']['x'],
                                                box['topleft']['y']),
                                                (box['bottomright']['x'],
                                                box['bottomright']['y']),
                                                (255,0,0), 6)
                    cv2.putText(self.img_out,box['label'],
                                  (box['topleft']['x'],
                                  box['topleft']['y']-12), 0 ,0.6,(255,0,0), 6//3)
            im_h = self.img_out.shape[0]
            im_w = self.img_out.shape[1]
            im_c = self.img_out.shape[2]


            if self.boxes:
                rospy.loginfo("boxes img_out.shape %d %d %d",im_h,im_w,im_c)
                rospy.loginfo("boxes: %s",self.boxes)
                #for debugging uncomment the instruction below to excit and llok at the boxes
                sys.exit("Exciting since we have boxes ")        
            else:
                rospy.loginfo("No boxes img_out.shape %d %d %d",im_h,im_w,im_c)
                             
        #Get classification
        return self.light_classifier.get_classification(self.img_out,self.boxes)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint close to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	#rospy.loginfo("process_traffic_lights()")
        light = None
        #if self.has_image == True:
	#rospy.loginfo(" tl_detector.py process_traffic_light() has image == True")
        #else: 
	#rospy.loginfo(" tl_detector.py process_traffic_light() has image == False")

        # find the closest visible traffic light (if one exists)
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            tl_position = self.find_next_stop_line(stop_line_positions)
	    #rospy.loginfo(" tl_detector.py process_traffic_light() tl position: %s ",tl_position)

       
        if self.has_image == True:
	    #rospy.loginfo("tl_detector.py process_traffic_lights() call YOLO since we have an image: %d",self.has_image)
            state = self.get_light_state(light)
            light_wp =[0] * 2
            return light_wp, state
        else:
            rospy.loginfo("Will not call YOLO no image: %d ",self.has_image)

        self.waypoints = None
	#rospy.loginfo("tl_detector.py process_traffic_lights()find closest UNKNOWN ")
        return -1, TrafficLight.UNKNOWN
    
    def get_euclidean_distance(self, pos1, pos2):
        #rospy.loginfo("get_euclidean_distance() ")
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 )


    def get_model_callback(self):
        #rospy.loginfo("get_model() options")
        options = {"model": "./cfg/tiny-yolo-udacity.cfg", "backup": "./ckpt/","load": 8987, "gpu": 1.0}
        #rospy.loginfo("get_model_callback() after options")
        model = TFNet(options)
        rospy.loginfo("get_model_callback() return model")
        return model

if __name__ == '__main__':
    try:
	rospy.loginfo("tl_detector.py main() instantiate TLDetector() ")

        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
