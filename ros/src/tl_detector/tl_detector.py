#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from light_classification.tl_classifier import TLClassifier
from waypoint_lib.waypoint_tracker import WaypointTracker

import tf as tf_ros
import math
import cv2
import yaml
STATE_COUNT_THRESHOLD = 3

class TLDetector(WaypointTracker):
    def __init__(self):
        WaypointTracker.__init__(self)

        rospy.init_node('tl_detector')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.current_pose_sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_array_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.previous_traffic_light_position = 0
        self.number_traffic_lights_passed = 0

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf_ros.TransformListener()
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        rospy.spin()
    def base_waypoints_cb(self, msg):
        WaypointTracker.base_waypoints_process(self, msg)
    def current_pose_cb(self, msg):
        self.pose = msg
    def traffic_array_cb(self, msg):
        self.lights = msg.lights
    
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
            light_wp = light_wp if state == TrafficLight.RED else -light_wp
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        # end of if self.state != state
        self.state_count += 1
    
    FAKED_LIGHT = True
    
    def get_light_state(self, light_index):
        """Determines the current color of the traffic light
    
        Args:
            light_index (TrafficLight): light to classify
    
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
    
        """
        if FAKED_LIGHT:
            return self.lights[light_index].state
        # end of if FAKED_LIGHT
    
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
        self.stop_line_positions = self.config['stop_line_positions']
    
        if (self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        # end of if (self.pose)
    
        #TODO find the closest visible traffic light (if one exists)
        # the index of the waypoint of the traffic light
        light_index, light_wp = self.find_closest_traffic_light(car_position)
    
        if light_index:
            state = self.get_light_state(light_index)
            return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN
    
    def find_closest_traffic_light(self, car_position_index):
        remaining_traffic_ligths = len(self.stop_line_positions)-self.number_traffic_lights_passed
        if 0 < remaining_traffic_ligths:
            dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
            # find the closest traffic light to the car's position
            traffic_light_index = self.previous_traffic_light_position
            d_shortest = dl(self.base_waypoints[car_position_index].pose.pose.position,
                            self.stop_line_positions[self.previous_traffic_light_position])
    
            for i in range(self.previous_traffic_light_position+1,
                           self.previous_traffic_light_position + remaining_traffic_ligths):
                d = dl(self.base_waypoints[car_position_index].pose.pose.position,
                self.stop_line_positions[i])
                if d < d_shortest:  # found the closest
                    d_shortest = d
                    traffic_light_index = i
                # end of if d < d_shortest
            # end of for i in range(self.previous_traffic_light_position, len(self.stop_line_positions)-self.number_traffic_lights_passed)
            self.previous_traffic_light_position = traffic_light_index
            self.number_traffic_lights_passed += 1
    
            # find the closest base_waypoint to the found traffic light.
            nearest_waypoint_for_the_light = car_position_index
            d_shortest = dl(self.base_waypoints[car_position_index].pose.pose.position, self.stop_line_positions[traffic_light_index])
            for j in range(car_position_index + 1, len(self.base_waypoints)):
                d = dl(self.base_waypoints[j].pose.pose.position, self.stop_line_positions[traffic_light_index])
                if d < d_shortest:
                    d_shortest = d
                    nearest_waypoint_for_the_light = j
                # end of if d < d_shortest
            # end of for j in range(car_position_index, len(self.base_waypoints)-car_position_index)
            return traffic_light_index, nearest_waypoint_for_the_light
        else:
            return None, None

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
