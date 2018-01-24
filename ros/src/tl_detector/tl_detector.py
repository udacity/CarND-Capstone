#!/usr/bin/env python
import cv2
import math
import numpy as np
import PIL.Image as PILImage
import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from light_classification.tl_classifier import TLClassifier
from styx_msgs.msg import Lane, TrafficLight, TrafficLightArray

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
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
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
        
        self.stop_line_positions = self.config['stop_line_positions']
        self.light_pos_waypoint = []
        self.car_position_index = None

        self.safe_for_light_distance = 18

        rospy.spin()

    def pose_cb(self, msg):
        """The call back function of the current car pose"""
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """This is the call_back function when we got the base way points""" 
        self.waypoints = waypoints.waypoints

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
        
        #Process traffic light status
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

    def get_closest_waypoint(self, pose, flag="light"):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_waypoint_dist = 100000
        closest_waypoint_ind = -1

        #Use loop to find closest one, based on https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        for i in range(0, len(self.waypoints)):
            if flag == "light":
                waypoint_distance = self.distance_point([self.waypoints[i].pose.pose.position.x,self.waypoints[i].pose.pose.position.y] \
                                                   , [pose[0], pose[1]])
            else:
                waypoint_distance = self.distance_point([self.waypoints[i].pose.pose.position.x,self.waypoints[i].pose.pose.position.y] \
                                                   , [pose.position.x, pose.position.y])
            if waypoint_distance <= closest_waypoint_dist:
                closest_waypoint_dist = waypoint_distance
                closest_waypoint_ind = i   

        return closest_waypoint_ind

    

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
         
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
    
    def distance_point(self,pos1,pos2):
        """
        Calculate the euclidian distance between two points
        """
        x = pos1[0] - pos2[0]
        y = pos1[1] - pos2[1]
        return math.sqrt(x*x + y*y)
        
        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Wait untill we got waypoints
        if self.waypoints is None or self.pose is None:
            return -1, TrafficLight.UNKNOWN

        light = None
        # List of positions that correspond to the line to stop in front of for a given intersection
        

        if len(self.light_pos_waypoint) == 0: #This part only calculate once
            for i in range(len(self.stop_line_positions)):
                #Fit the stopline points to the base way points
                light_pos = self.get_closest_waypoint(self.stop_line_positions[i],"light")
                self.light_pos_waypoint.append(light_pos)


        
        self.car_position_index = self.get_closest_waypoint(self.pose.pose,"car")
        if self.car_position_index > max(self.light_pos_waypoint):
            light_wp_id = min(self.light_pos_waypoint)
        else:
            light_delta = self.light_pos_waypoint[:]
            # Calculate the distance between all Light and the current car 
            light_delta[:] = [x - self.car_position_index for x in light_delta]
            # Find the nearest light in front of the car
            light_wp_id = min(i for i in light_delta if i >= 0) + self.car_position_index
        
        # Map back to the stopline
        light_ind = self.light_pos_waypoint.index(light_wp_id)
        light = self.stop_line_positions[light_ind]

        light_distance = self.distance_point([light[0],light[1]], \
                        [self.waypoints[self.car_position_index].pose.pose.position.x,self.waypoints[self.car_position_index].pose.pose.position.y] )
        
        if light and light_distance < self.safe_for_light_distance:
            # Brake the car within the safe distance
            state = self.get_light_state(light)
            return light_wp_id, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
