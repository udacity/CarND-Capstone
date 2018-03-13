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
import math
import time

STATE_COUNT_THRESHOLD = 3
GROUND_TRUTH_PASS = True  
#Variable to use ground truth data until the classifier has been developed

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

		#Private Parameter to differentiate simulation or Site Launch  
		#Default - site launch
        self.isSimulation = bool(rospy.get_param('~isSimulation', False))
	
		#Variable Definitions
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # tl_detection node subscribes to:
    	#/base_waypoints provides the complete list of waypoints for the course.
		#/current_pose can be used used to determine the vehicle's location.
		#/image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
		#/vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.

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

		#[Meenu] - Commented the following temporarily 
        #self.bridge = CvBridge()
        #self.light_classifier = TLClassifier()
        #self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
	
        rospy.loginfo('Red: %s', TrafficLight.RED)
        rospy.loginfo('Yellow: %s', TrafficLight.YELLOW)
        rospy.loginfo('Green: %s', TrafficLight.GREEN)
        rospy.loginfo('Unknown: %s', TrafficLight.UNKNOWN)

        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_waypoints = []


        rospy.spin()

    def pose_cb(self, msg):
	"""Callback fuction for vehicle's location."""
        self.pose = msg

    def waypoints_cb(self, waypoints):
	"""Callback fuction for list of all waypoints."""
        self.waypoints = waypoints
        self.stop_line_waypoints = []
        for pts in self.stop_line_positions:
            sl_wp = self.get_closest_waypoint(pts[0], pts[1], self.waypoints)
            self.stop_line_waypoints.append(sl_wp)


    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index 
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        if GROUND_TRUTH_PASS:
			light_wp, state = self.process_ground_truth_lights()
        else:        
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


    def get_closest_waypoint(self, pos_x, pos_y, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        wp_id = None
        wps = waypoints.waypoints

        x = pos_x
        y = pos_y

        wpx = wps[0].pose.pose.position.x
        wpy = wps[0].pose.pose.position.y

        min_dist = math.sqrt((x - wpx)**2 + (y - wpy)**2)

        # check all the waypoints to see which one is the closest to our current position
        for i, waypoint in enumerate(wps):
            wps_x = waypoint.pose.pose.position.x
            wps_y = waypoint.pose.pose.position.y
            dist = math.sqrt((x - wps_x)**2 + (y - wps_y)**2)
            if (dist < min_dist): #we found a closer wp
                wp_id = i     # we store the index of the closest waypoint
                min_dist = dist     # we save the distance of the closest waypoint

		# returns the index of the closest waypoint

        return wp_id




    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
        	car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def process_ground_truth_lights(self):
        """Finds closest traffic light in the ground truth data and returns the state of the light

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
	
        ntl_wp = -1
        ntl_state =TrafficLight.UNKNOWN

        if(self.pose):
			car_position = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y, self.waypoints)
        if car_position:
			for tl in self.lights: 	
				nearest_waypoint = self.get_closest_waypoint(tl.pose.pose.position.x,tl.pose.pose.position.y, self.waypoints)
				if nearest_waypoint > car_position:
					ntl_wp = nearest_waypoint
					ntl_state = tl.state
					break
        stop_line = -1 
        if ntl_wp > 0:
			#stop line nearest to the nearest light
			stop_distance = 10000
			for position in self.stop_line_waypoints:
				if position < ntl_wp:
					if ((ntl_wp - position) < stop_distance):
						stop_distance = (ntl_wp - position)	
						stop_line = position
        if stop_line > 0:
			rospy.loginfo('Light state: %s',ntl_state)
			rospy.loginfo('Stop line: %f',stop_line)			
			return stop_line, ntl_state 
        else:
	        rospy.loginfo('Light state: %s',TrafficLight.UNKNOWN)
	        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
