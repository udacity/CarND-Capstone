#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_model_classifier import TLModelClassifier
import tf
import cv2
import numpy as np
import yaml
import time
import os

STATE_COUNT_THRESHOLD = 3

"""
DIST_FROM_A_LIGHT_STOP_LINE is used to decide if there is a light stop line ahead close enough that the car needs to execute light detection to decide whether to brake for red light or maintain current speed to pass a green light. Eg. when DIST_FROM_A_LIGHT_STOP_LINE is 65m, we would not worry about a light that's 500m ahead. Only when closest way point is with 65m to the target light stop line, would the car start light category detection. Note the decision would require a consecutive STATE_COUNT_THRESHOLD frames to decide. More time is also required for the car to decelerate to a complete stop before the light stop line.

Max sim highway speed is 40km/h, which is about 11m/s
/current_pose topic refreshes at 50Hz. 
/image_color topic refreshes at 10Hz. So we can update light detection result every 0.1s.
It will take STATE_COUNT_THRESHOLD * 0.1s = 0.3 before the car determines there is red light ahead. During that 0.3s, the can drove about 3.3 meters. The max dec given in dbw lauch file is -5m/s^2, then it will take 2.2s to drop velocity to zero, which implies another 0.5*2.2*5^2 = 27.5 meters out. In conclusion when the distance from the light stop line ahead is smaller than 27.5 + 3.3 + 3 (since pose might be at center of car, needs to consider length of car. Lincoln seems huge. Thus assume car length to be 3 meters to be safe) = 34 meters, we should definitely do light detection. Given it some more buffer, setting DIST_FROM_A_LIGHT_STOP_LINE to 65 meters. This might be passed in as a parameter given different configs such as speed limit or max deceleration, etc.
"""
DIST_FROM_A_LIGHT_STOP_LINE = 65.0

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        # All stop line positions are fixed. Just need to load once
        self.stop_line_positions = self.config['stop_line_positions']
        rospy.loginfo('stop_line_positions:\n {}'.format(self.stop_line_positions))
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if(hasattr(self, 'light_classifier') == False):
            if(len(self.lights) == 1):
                rospy.loginfo('Using model based classifier.')
                self.light_classifier = TLModelClassifier()
            else:
                rospy.loginfo('Using color based classifier.')
                self.light_classifier = TLClassifier()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
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
        if(self.last_wp > -1):
            rospy.loginfo_throttle(1, 'Red light stop line waypoint index:{}'.format(self.last_wp))

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Ref: https://codereview.stackexchange.com/questions/28207/finding-the-closest-point-to-a-list-of-points
        idx = -1
        #Return if way points are empty
        if self.waypoints:
            pos_np_array = np.asarray([pose.position.x, pose.position.y])
            dist = np.sqrt(np.sum((np.asarray([(w.pose.pose.position.x, w.pose.pose.position.y) for w in self.waypoints.waypoints]) - pos_np_array)**2, axis=1))
            idx = np.argmin(dist)
            # rospy.loginfo_throttle(10, 'get_closest_waypoint\nPos: {}\nIndex:{}\nDistance: {}'.format(pos_np_array, idx, dist[idx]))
        return idx

    def get_closest_light_stop_line_idx(self, pose, detect_range):
        """Identifies the closest stop line position to the given position with in 
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): way point position to match a stop line position to
	    detect_range (Double): min distance to light to start light detection
        Returns:
            int: index of the closest stop line position in self.waypoints that is within detect_range

        """
        result = -1
        idx = -1
        #Return if light stop lines are empty
        if self.lights:
            pos_np_array = np.asarray([pose.position.x, pose.position.y])
            dist = np.sqrt(np.sum((np.asarray([(l[0], l[1]) for l in self.stop_line_positions]) - pos_np_array)**2, axis=1))
            idx = np.argmin(dist)
            if(dist[idx] > detect_range):
                result = -1;
            else:
                result = idx

        #rospy.loginfo_throttle(3, 'get_closest_light_stop_line_idx\nPos: {}\nIndex:{}\nDistance: {}\nReturn: {}'.format(pos_np_array, idx, dist[idx], result))
        return result

    # Based on observation, when car is advancing waypoint index is ascending
    # So when index is a little bit larger (within 1/8 of total waypoint number), we can conclude it's in front.
    def waypoint_is_ahead_of(self, idx_a, idx_b):
    	waypoint_num = len(self.waypoints.waypoints)
    	minimum_waypoint_num = waypoint_num/8 # only return meaningful value if diff  between two waypoint index passed in is smaller than this value
    	is_ahead = idx_a > idx_b and idx_a - idx_b < minimum_waypoint_num or idx_a < idx_b and waypoint_num - idx_b + idx_a < minimum_waypoint_num # Considering wrap-around case, if the distance between a and b is less than 1/8 of the loop
    	#rospy.loginfo_throttle(3, 'waypoint_is_ahead_of\nidx_a: {}\nidx_b:{}\nIs Ahead: {}'.format(idx_a, idx_b, is_ahead))
    	return is_ahead

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        state = None
        if(not self.has_image):
            self.prev_light_loc = None
            return False
	
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #self.save_training_data(cv_image, light.state)
	    
        #Get classification
        if(hasattr(self, 'light_classifier')):
            state = self.light_classifier.get_classification(cv_image)
        #rospy.loginfo_throttle(3, 'get_light_state\nResult from Classifiter: {}\nGround Truth:{}\nMatch: {}'.format(state, light.state, state == light.state))
    	if(state != light.state and light.state == TrafficLight.RED):
    	    rospy.logwarn("RED NOT CAUGHT!")
    	    debug_data_dir = "debug_data/"
    	    if not os.path.exists(debug_data_dir):
                os.makedirs(debug_data_dir)
            save_path =  os.path.join(debug_data_dir, "{}_red.jpg".format(time.time())) if state == TrafficLight.RED else os.path.join(debug_data_dir, "{}_not_red.jpg".format(time.time()))
    	    cv2.imwrite(save_path, cv_image)
    	return state

    def save_training_data(self, img, state):
    	training_data_dir = "training_data/"
    	red_data_folder = os.path.join(training_data_dir, "red")
    	not_red_data_folder = os.path.join(training_data_dir, "not_red")
    	if not os.path.exists(red_data_folder):
                os.makedirs(red_data_folder)
    	if not os.path.exists(not_red_data_folder):
                os.makedirs(not_red_data_folder)

    	save_path =  os.path.join(red_data_folder, "{}_red.jpg".format(time.time())) if state == TrafficLight.RED else os.path.join(not_red_data_folder, "{}_not_red.jpg".format(time.time()))
    	rospy.loginfo_throttle(3, 'save_training_data\nPath: {}'.format(save_path))
    	cv2.imwrite(save_path, img)
	    

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
    	light_wp = -1
    	state = TrafficLight.UNKNOWN
        # List of positions that correspond to the line to stop in front of for a given intersection
        if(self.pose):
	    # stop line index closet to car pos within DIST_FROM_A_LIGHT_STOP_LINE. Only when distance to light stop line is within DIST_FROM_A_LIGHT_STOP_LINE, will a valid index be returned. Else will get -1
	    idx_of_closest_light_stop_line = self.get_closest_light_stop_line_idx(self.pose.pose, DIST_FROM_A_LIGHT_STOP_LINE)
	    if(idx_of_closest_light_stop_line > -1):
	    	# pos of the light stop line closest to current location of car. Note it's not necessarily ahead of car
	    	light_stop_pos = Pose()
	    	light_stop_pos.position.x = self.stop_line_positions[idx_of_closest_light_stop_line][0]
	    	light_stop_pos.position.y = self.stop_line_positions[idx_of_closest_light_stop_line][1]

	        # waypoint index closest to light stop line
	        idx_closest_waypoint_to_light_stop_line = self.get_closest_waypoint(light_stop_pos)
	        # waypoint index closest to car pos
	        idx_closest_waypoint_to_car = self.get_closest_waypoint(self.pose.pose)

	        if (idx_closest_waypoint_to_car > -1
		    and idx_closest_waypoint_to_light_stop_line > -1
		    and self.waypoint_is_ahead_of(idx_closest_waypoint_to_light_stop_line, idx_closest_waypoint_to_car)):
		        # Use the camera info now to detect if it's red light. If so, publish the stop line waypoint
		        light_wp = idx_closest_waypoint_to_light_stop_line
		        light = self.lights[idx_of_closest_light_stop_line] # Used as ground truth, not for classification
		        if light:
		            state = self.get_light_state(light)
        #rospy.loginfo_throttle(3, 'process_traffic_lights\nlight: {}\nlight_wp:{}\nstate: {}'.format(light, light_wp, state))
        return light_wp, state
	
	
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')