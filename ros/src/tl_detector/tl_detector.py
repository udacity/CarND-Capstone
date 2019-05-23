#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_unet_classifier import TLUnetClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import time
import thread

import numpy as np
from math import pow, sqrt

from keras.models import load_model
from keras import backend as K



STATE_COUNT_THRESHOLD = 3

SMOOTH = 1.

def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + SMOOTH) / (K.sum(y_true_f) + K.sum(y_pred_f) + SMOOTH)

def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)
    

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        #self.lights = []
        self.number_of_detected_lights = 0
        self.has_image = False
        self.thread_working = False

        self.frame_count = 0
        self.detector_model = None


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # UNET Classifier Setup  
        self.light_unet_classifier = TLUnetClassifier()
        tl_classification_model = "unet_models/tl_classifier_simulator.h5"
        model = load_model(tl_classification_model)
        classifier_resize_height = 64
        classifier_resize_width = 32
        self.light_unet_classifier.setup_classifier(model, classifier_resize_width, classifier_resize_height)
        self.invalid_class_number = 3
        
        # UNET Detector setup
        tl_detection_model = "unet_models/tl_detector_simulator.h5"
        self.detector_model = load_model(tl_detection_model, 
                               custom_objects={'dice_coef_loss': dice_coef_loss, 'dice_coef': dice_coef })
        self.detector_model._make_predict_function()
        
        self.resize_height = 96
        self.resize_width = 128  
        self.resize_height_ratio = 600/float(self.resize_height)
        self.resize_width_ratio = 800/float(self.resize_width)
        self.color_mode = 'rgb8'
        self.middle_col = self.resize_width/2
        self.is_carla = False
        self.projection_threshold = 2
        self.projection_min = 200
        self.distance_to_tl_threshold = 67.0

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp_idx = -1
        self.state_count = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string,Loader=yaml.FullLoader)
        self.stop_line_positions = self.config['stop_line_positions']

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.wl_debug_lights = 0
        self.lights = []
        
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        mode = "unet"
        #mode = "mobilenet"    
        if mode == "mobilenet":
            # mobilenet model
            sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
            rospy.spin()
        else:
            # u-net model
            sub6 = rospy.Subscriber('/image_color', Image, self.image_unet_cb)
            
            detector_rate = 6
            detector_rate = rospy.Rate(detector_rate) #detector_rate: 6
            while not rospy.is_shutdown():
                self.unet_find_traffic_lights()
                detector_rate.sleep()

    def traffic_lights_cb(self, msg):
        self.lights = msg.lights

        if self.wl_debug_lights < 1:
            self.wl_debug_lights += 1
            rospy.loginfo("[tl_detector] %s traffic_lights_cb stoplines.len=%s ", 
                            self.wl_debug_lights, len(self.config['stop_line_positions']))
            for n in range(len(self.config['stop_line_positions'])):
                rospy.loginfo("[tl_detector] stop_line_positions[%s] : x=%s, y=%s ", n, 
                                self.config['stop_line_positions'][n][0],
                                self.config['stop_line_positions'][n][1])
                                                
            rospy.loginfo("[tl_detector] %s traffic_lights_cb self.lights.len=%s ", 
                        self.wl_debug_lights, len(self.lights))
            for n in range(len(self.lights)):
                rospy.loginfo("[tl_detector] lights[%s] : [%s, %s], ", n, 
                                self.lights[n].pose.pose.position.x,
                                self.lights[n].pose.pose.position.y)

    #################################### U-Net Part #######################

    def unet_find_traffic_lights(self):
        stop_wp_idx, state = self.process_traffic_lights_unet()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self._pass_threshold():
            self.last_state = self.state
            stop_wp_idx = stop_wp_idx if state == TrafficLight.RED else -1
            
            self.last_wp_idx = stop_wp_idx
            self.upcoming_red_light_pub.publish(Int32(self.last_wp_idx))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp_idx))

        self.state_count += 1
        rospy.loginfo("[UNET] %s publish traffic_waypoint: %s. state=%s", 
                                        self.state_count, self.last_wp_idx, self.state)
        

    def dist_to_point(self, pose, wp_pose):
        x_squared = pow((pose.position.x - wp_pose.position.x), 2)
        y_squared = pow((pose.position.y - wp_pose.position.y), 2)
        dist = sqrt(x_squared + y_squared)
        return dist

    def get_closest_wp_idx(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypoints : points where to look for closest one

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_dist = float("inf")
        closest_wp_idx = -1

        if not waypoints:
            rospy.logwarn("[TL_DETECTOR] No waypoints given.")
        else:
            for idx, wp in enumerate(waypoints):
                dist = self.dist_to_point(pose, wp.pose.pose)
                if(dist < min_dist):
                    min_dist = dist
                    closest_wp_idx = idx
        return closest_wp_idx

    def _pass_threshold(self):
        if self.state == TrafficLight.YELLOW:
            return self.state_count >= STATE_COUNT_THRESHOLD - 1
        return self.state_count >= STATE_COUNT_THRESHOLD



    def process_traffic_lights_unet(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
       
        self.base_waypoints = None     int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        if (self.pose is not None) and (self.has_image) and (self.base_waypoints is not None):
            tl_idx = self.get_closest_wp_idx(self.pose.pose, self.lights)
            rospy.logdebug("[UNET] Closest TL idx: %s.", tl_idx)
            if (tl_idx >= 0):
                tl_wp = self.lights[tl_idx]
                stop_line = self.config['stop_line_positions'][tl_idx]
                
                # Convert stop_line to pseudo pose object for easier handling
                stop_line_pose = Pose()
                stop_line_pose.position.x = stop_line[0]
                stop_line_pose.position.y = stop_line[1]
                    
                stop_wp_idx = self.get_closest_wp_idx(stop_line_pose, self.base_waypoints.waypoints)
                if (stop_wp_idx == -1):
                    rospy.logdebug("[UNET] Unable to determine valid TL idx.")
                    return -1, TrafficLight.UNKNOWN
                else:
                    rospy.logdebug("[UNET] Closest WP idx: %s.", stop_wp_idx)
                    
                closest_stop_wp = self.base_waypoints.waypoints[stop_wp_idx]
                stop_tl_dist = self.dist_to_point(closest_stop_wp.pose.pose, tl_wp.pose.pose)
                car_stop_dist = self.dist_to_point(self.pose.pose, stop_line_pose)
                
                state = TrafficLight.UNKNOWN
                                    
                if (car_stop_dist < self.distance_to_tl_threshold):
                    start = time.time()
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, self.color_mode)
                    tl_image = self.detect_traffic_light(cv_image)
                    end1 = time.time()

                    if tl_image is not None:
                        rospy.logdebug("[UNET] Detection Time:%f s", end1 - start)
                        
                        state = self.light_unet_classifier.get_classification(tl_image)
                        state = state if (state != self.invalid_class_number) else TrafficLight.UNKNOWN
                        rospy.logdebug("[UNET] Nearest TL-state is: %s", state)
                        end2 = time.time()
                        rospy.logdebug("[UNET] Classify Time:%f s", end2 - end1)
                    else:
                        rospy.logdebug("[UNET] No TL is detected ")
                else:
                    rospy.logdebug("[UNET] Next TL too far yet.")
                    state = TrafficLight.UNKNOWN
        
                car_wp_idx = self.get_closest_wp_idx(self.pose.pose, self.base_waypoints.waypoints)
                tl_wp_idx = self.get_closest_wp_idx(tl_wp.pose.pose, self.base_waypoints.waypoints)
                rospy.loginfo("[UNET] XYZ Car position(%s): x-%s, y-%s; car_stop_dist=%s", 
                                car_wp_idx, self.pose.pose.position.x, self.pose.pose.position.y, car_stop_dist)
                rospy.loginfo("[UNET] XYZ Stop line   (%s): x-%s, y-%s; stop_tl_dist=%s", 
                                stop_wp_idx, stop_line[0], stop_line[1], stop_tl_dist)
                rospy.loginfo("[UNET] XYZ TrafficLight(%s): x-%s, y-%s; Upcoming TL ID:%s, State:%s", 
                                tl_wp_idx, tl_wp.pose.pose.position.x, tl_wp.pose.pose.position.y, tl_idx, state)
                return stop_wp_idx, state
        
            else:
                rospy.logwarn("[UNET] No trafic light found!")
                return -1, TrafficLight.UNKNOWN
        else:
            rospy.logwarn("[UNET] No EGO position available!")
            return -1, TrafficLight.UNKNOWN

        #self.base_waypoints = None
        return -1, TrafficLight.UNKNOWN

    def _extract_image(self, pred_image_mask, cv_image):
        # pred_image_mask=(96,128,1), cv_image=(600, 800, 3)
        if (np.max(pred_image_mask) < self.projection_min):
            return None

        row_projection = np.sum(pred_image_mask, axis = 1)
        row_index =  np.argmax(row_projection)
        rospy.logdebug("row_projection.shape=%s, row_index=%s", row_projection.shape, row_index)   # (96,128,1)

        if (np.max(row_projection) < self.projection_threshold):
            return None

        zero_row_indexes = np.argwhere(row_projection <= self.projection_threshold)
        top_part = zero_row_indexes[zero_row_indexes < row_index]
        top = np.max(top_part) if top_part.size > 0 else 0
        bottom_part = zero_row_indexes[zero_row_indexes > row_index]
        bottom = np.min(bottom_part) if bottom_part.size > 0 else self.resize_height

        roi = pred_image_mask[top:bottom,:]
        column_projection = np.sum(roi, axis = 0)

        if (np.max(column_projection) < self.projection_min):
            return None

        non_zero_column_index = np.argwhere(column_projection > self.projection_min)

        index_of_column_index =  np.argmin(np.abs(non_zero_column_index - self.middle_col))
        column_index = non_zero_column_index[index_of_column_index][0]

        zero_colum_indexes = np.argwhere(column_projection == 0)
        left_side = zero_colum_indexes[zero_colum_indexes < column_index]
        left = np.max(left_side) if left_side.size > 0 else 0
        right_side = zero_colum_indexes[zero_colum_indexes > column_index]
        right = np.min(right_side) if right_side.size > 0 else self.resize_width
        
        return cv_image[int(top*self.resize_height_ratio):int(bottom*self.resize_height_ratio), 
                        int(left*self.resize_width_ratio):int(right*self.resize_width_ratio)] 

    def detect_traffic_light(self, cv_image):
        resize_image = cv2.cvtColor(cv2.resize(cv_image, (self.resize_width, self.resize_height)), cv2.COLOR_RGB2GRAY)
        rospy.logdebug("resize_image.shape 1=%s, cv_image=%s", resize_image.shape, cv_image.shape)          
        # resize_image.shape 1=(96, 128), cv_image=(600, 800, 3)
        resize_image = resize_image[..., np.newaxis]
        rospy.logdebug("resize_image.shape 2=%s", resize_image.shape)   # (96,128,1)
        if self.is_carla:
            mean = np.mean(resize_image) # mean for data centering
            std = np.std(resize_image) # std for data normalization

            resize_image -= mean
            resize_image /= std

        image_mask = self.detector_model.predict(resize_image[None, :, :, :], batch_size=1)[0]
        image_mask = (image_mask[:,:,0]*255).astype(np.uint8)        
        rospy.logdebug("image_mask.shape 3=%s", resize_image.shape)     # resize_image=(96,128,1), cv_image=(600, 800, 3)
        return self._extract_image(image_mask, cv_image)
        

    
    def pose_cb(self, msg):
        self.pose = msg

    def base_waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] 
                                    for wp in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    '''
    def traffic_cb(self, msg):
        self.lights = msg.lights
    '''

    def image_unet_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg



    #################################### Mobile Net Part #######################


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo("Image_cb")
        self.has_image = True

        if not self.thread_working:
            self.thread_working = True
            
            self.camera_image = msg

            thread.start_new_thread( self.detect_tl, ())
            #box_image = self.light_classifier.detect_traffic_lights(cv_image)

            #pic_filename = "./result/%08d.png"%self.frame_count
            #cv2.imwrite(pic_filename, box_image)
            #self.frame_count += 1

            #image_message = self.bridge.cv2_to_imgmsg(box_image, encoding="passthrough")           
            
            #self.upcoming_red_light_pub.publish(Image(image_message))           
            

    def detect_tl(self):
        #rospy.loginfo("Detection start")
        stop_wp_idx, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_wp_idx = stop_wp_idx if state == TrafficLight.RED else -1
            self.last_wp_idx = stop_wp_idx
            self.upcoming_red_light_pub.publish(Int32(stop_wp_idx))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp_idx))

        self.state_count += 1

        self.thread_working = False
            

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #For test in simulator
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        return light.state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
       
        self.base_waypoints = None     int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """     
        start = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        self.number_of_detected_lights = self.light_classifier.detect_traffic_lights(cv_image)
        
        end1 = time.time()
        rospy.logdebug("Mobilenet Detection Time:%f s, Num of lights %d", 
                        end1 - start, self.number_of_detected_lights)

        closest_light = None
        closet_stop_line_wp_idx = None
        state = None
        
        if self.number_of_detected_lights > 0:
            state = self.light_classifier.get_classification()
            end2 = time.time()
            rospy.logdebug("Mobilenet classify Time:%f s", end2 - end1) 
            # state = self.light_unet_classifier.get_classification(tl_image)
            rospy.loginfo("-------------- light state %d---------------- ", state)

            if state == TrafficLight.RED:
                if(self.pose):
                    car_pose_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

                    smallest_wp_dist = len(self.base_waypoints.waypoints)
                    for i in range(len(self.stop_line_positions)):
                        stop_line_point = self.stop_line_positions[i]
                        stop_line_point_wp_idx = self.get_closest_waypoint(stop_line_point[0], stop_line_point[1])

                        dist = stop_line_point_wp_idx - car_pose_wp_idx
                        if dist >= 0 and dist < smallest_wp_dist:
                            smallest_wp_dist = dist
                            closet_stop_line_wp_idx = stop_line_point_wp_idx
                    rospy.loginfo("Closet waypoint is %d:", closet_stop_line_wp_idx)        
                    return closet_stop_line_wp_idx, state

        #Simulation code start
        '''
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.base_waypoints.waypoints)
            for i in range(len(self.lights)):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = self.lights[i]
                    line_wp_idx = temp_wp_idx
        '''
        #Simulation code end
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
