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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        '''
	light_state = 4 #Default to UNKNOWN
	line_index = self.get_closest_waypoint(self.pose.pose)
	if(len(self.lights) >= 1):
		light_state = self.lights[line_index].state
	if(light_state == 0): # If red light send topic of index
		self.upcoming_red_light_pub.publish(Int32(line_index))
	else: #If the light is not red
            self.upcoming_red_light_pub.publish(Int32(-1))
        '''
            
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
	#print("recieved image")
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        print('index = {} state = {}'.format(light_wp, state))
        if(light_state == 0): # If red light send topic of index
		self.upcoming_red_light_pub.publish(Int32(line_index))
	else: #If the light is not red
            self.upcoming_red_light_pub.publish(Int32(-1))
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        
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
        '''
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        stop_line_positions = self.config['stop_line_positions']
        
        minDist = 9999999
        minIndex = -1
        for i in range(len(stop_line_positions)):   
            x = stop_line_positions[i][0]
            y = stop_line_positions[i][1]
            car_x = pose.position.x
            car_y = pose.position.y

            # Get the yaw rate from the cars orientation
            quaternion = ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)            
            yaw = euler[2]

            # Shift the waypoint to be in the cars frame 
            shiftX = x - car_x
            shiftY = y - car_y
            
            newX = shiftX * math.cos(0 - yaw) - shiftY * math.sin(0 - yaw)
            newY = shiftX * math.sin(0 - yaw) - shiftY * math.cos(0 - yaw)

            if(newX >= 0): #Point is in front of car
                dist = math.sqrt( (x - car_x)**2 + (y - car_y)**2 ) # Find the distance to the point
                
                if(dist < minDist):                    
                    minDist = dist
                    minIndex = i

        #print('min_distance_i={} min_distance={}'.format(minIndex, minDist))
        return minIndex

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        simulation = true
        if(simulation):
            return light.state
        else:
            return 4
        '''
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
	'''
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        
        line_index = self.get_closest_waypoint(self.pose.pose)
        light_state = 4
	if(len(self.lights) >= 1):
		light_state = get_light_state(self, self.lights[line_index])#.state
	return line_index, light_state
    
	#if(light_state == 0): # If red light send topic of index                
	#	self.upcoming_red_light_pub.publish(Int32(line_index))
	#else: #If the light is not red
        #   self.upcoming_red_light_pub.publish(Int32(-1))
'''
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
