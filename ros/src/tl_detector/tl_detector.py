#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import time

STATE_COUNT_THRESHOLD = 3
LIGHT_MINIMUM_DETECTION_DISTANCE = 300 #defines min distance to look for traffic lights ahead
LIGHT_CLASSIFIER_MODE = 0 # 0 = classifier is ON, 1 = classifier is off, using simulator data, 2 = classifier is OFF, gathering camera images and labels for training NN
PATH_TRAINING_SAMPLES = '/home/student/catkin_ws/CarND-Capstone/data/TRAINING_DATA/'

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
	self.waypoints_2d = None
	self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb) #camera data

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

	# as result, we publish in /traffic_waypoint one int32 value
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

	#color ID printing
	rospy.loginfo('Red ID: %s, Yellow ID: %s, Green ID: %s, Unkwn ID: %s', TrafficLight.RED,TrafficLight.YELLOW,TrafficLight.GREEN,TrafficLight.UNKNOWN)

        rospy.spin()


    def pose_cb(self, msg):
	#callback for car's position
        self.pose = msg

    def waypoints_cb(self, waypoints):
	#callback for waypoints (full track)
        self.waypoints = waypoints

	if not self.waypoints_2d:
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:  msg (Image): image from car-mounted camera
        """
        #rospy.loginfo("image capture")
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose_x, pose_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args: pose (Pose): position to match a waypoint to
        Returns: int: index of the closest waypoint in self.waypoints
        """

        #TODO implement

        closest_idx = self.waypoint_tree.query([pose_x,pose_y],1)[1]
        return closest_idx



    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:  light (TrafficLight): light to classify
        Returns: int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
	#We use realtime detection and classification using the camera
        if LIGHT_CLASSIFIER_MODE == 0:
            if(not self.has_image):
                self.prev_light_loc = None
                return False

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            #Get classification
            return self.light_classifier.get_classification(cv_image)

        #We use the simulator light state only for testing other nodes
        elif LIGHT_CLASSIFIER_MODE == 1:
        
            if(not self.has_image):
                self.prev_loc_light = None
                return False
            
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            #Get classification
            classi = self.light_classifier.get_classification(cv_image)
            rospy.loginfo('Using Simulator, but Classifier says {}'.format(classi))
        
        
            return light.state
        #We use simulator light state to gather images+labels to train network
        else:
            if(not self.has_image):
                self.prev_light_loc = None
                return False

            save_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            # light.state 0red, 1yellow, 2green, 4unknown
            save_image = cv2.resize(save_image, (400,300))
            cv2.imwrite(PATH_TRAINING_SAMPLES + 'lightstate_{}_time_{}.png'.format(light.state, time.time()), save_image)
            rospy.loginfo('--------> saved image %s', time.time())
            return light.state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #current car wp
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                #iterates around stop line waypoints and picks closest point's index]
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                #find closest stop line waypoint index
                d = temp_wp_idx -car_wp_idx
                if d > 0.0 and d < diff and d < LIGHT_MINIMUM_DETECTION_DISTANCE: 
                    #pick closest light position if distance is <250m
                    #otherwise =None so we do not call get_light_state method
                    #to save useless CPU calculations. We are allowed to know light 	                
                    #positions, so classifier is used to color detection :)
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx	

        if closest_light:
            state = self.get_light_state(closest_light)
            rospy.loginfo('received from get_light_state: {}'.format(state))

            if (state == 0):
                lightcolor = 'RED'
            elif (state == 1):
                lightcolor = 'YELLOW'
            elif (state == 2):
                lightcolor = 'GREEN'
            elif (state == 4):
                lightcolor = 'UNKNOWN'

            #printout some logs in case of light detection in front, otherwise no printing
            rospy.loginfo('******* LIGHT DETECTED **************')
            rospy.loginfo('car position WP is    	%s', car_wp_idx)
            rospy.loginfo('linestop position WP is 	%s', line_wp_idx)
            rospy.loginfo('ligh color is		%s', lightcolor)
            rospy.loginfo('distance to stop is 	%s', diff)

            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
