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
import copy

STATE_COUNT_THRESHOLD = 3


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

        # todo fix: get_param fails with key not found exception
        # config_string = rospy.get_param("/traffic_light_config")
        # self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        #self.dbg_flag = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        """
        i = 0
        if self.waypoints != None and self.dbg_flag == 0:
        	i = 0
        	for light in self.lights:
        		wid = self.get_closest_waypoint(light.pose.pose)
        		print "light ", i, " waypoint ", wid
        		i = i +1
        	self.dbg_flag = 1
        """

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        ds = 10000000.0
        wy_id = -1
        i = 0
        """
        for wyp in self.waypoints.waypoints:
        	dx = pose.position.x - wyp.pose.pose.position.x
        	dy = pose.position.y - wyp.pose.pose.position.y
        	t = math.sqrt(dx*dx + dy*dy)
        	if t < ds:
        		ds = t
        		wy_id = i
        	i = i +1
        """
        wyp_c = []
        for wyp in self.waypoints.waypoints:
        	dx = math.fabs(pose.position.x - wyp.pose.pose.position.x)
        	dy = math.fabs(pose.position.y - wyp.pose.pose.position.y)
        	if dx < 10.0 or dy < 10.0:
        		wyp_c.append(i)
        	i = i +1
        	
        for w_i in wyp_c:
        	wy_pose = self.waypoints.waypoints[w_i].pose.pose.position
        	dx = pose.position.x - wy_pose.x
        	dy = pose.position.y - wy_pose.y
        	t = math.sqrt(dx*dx + dy*dy)
        	if t < ds:
        		ds = t
        		wy_id = w_i
        
        return wy_id


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def conv2car_frame(self, w_x, w_y):
		"""convert the world frame coordinates to local car frame coordinates (2D)
		"""
		pos = self.pose.pose.position
		qor = self.pose.pose.orientation
		quaternion = (qor.x, qor.y, qor.z, qor.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		theta = euler[2]
	
		x = (w_x - pos.x)*math.cos(theta)+(w_y - pos.y)*math.sin(theta)
		y = (w_x - pos.x)*(-math.sin(theta))+(w_y - pos.y)*math.cos(theta)
	
		return (x, y)
	
    def get_closest_lights(self, light_positions):
		"""find the closest light
		"""
		min_ds = 10000000.0
		n = 10
	
		i  = 0
		st = 4
		for light_position in light_positions:
			l_wx = light_position[0]
			l_wy = light_position[1]
			x, y = self.conv2car_frame(l_wx, l_wy)

			if x > 0:
				ds = math.sqrt(x*x + y*y)
				if ds < min_ds:
					min_ds = ds
					n = i
			i = i +1
	
		return (n, min_ds, light_positions[n][0], light_positions[n][1])

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

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = self.config['light_positions']
        
        #TODO find the closest visible traffic light (if one exists)
        if self.waypoints and self.pose:
            car_position = self.get_closest_waypoint(self.pose.pose)
            li, ds, lx, ly = self.get_closest_lights(light_positions)
            #print "closest way light: ", li, ds, lx, ly
            # TODO: call get_light_state(light)
            if ds < 50:
            	light_pose = copy.deepcopy(self.lights[li].pose.pose)
            	lwp = self.get_closest_waypoint(light_pose)
            	state = self.lights[li].state
            	#print "light waypoint is ", lwp, " state ", state
            	return lwp, state

        """if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        """
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
