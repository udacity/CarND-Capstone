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
def waypoint_to_light_f(lights_to_waypoints, base_waypoints_num):
    # implementation
    waypoint_to_light = {}
    light_next = 0

    for waypoint_index in range(base_waypoints_num):
        for light_index in range(light_next, len(lights_to_waypoints)):
            if waypoint_index < lights_to_waypoints[light_index]:
                waypoint_to_light[waypoint_index] = (light_index, lights_to_waypoints[light_index])
                break
            elif lights_to_waypoints[-1] <= waypoint_index:
                waypoint_to_light[waypoint_index] = (None, None)
                break
            # end of if waypoint_index <= lights_to_waypoints[light_index]
            light_next = light_index
        # end of for light_index in range(len(lights_to_waypoints))
    # end of for i in range(base_waypoints_num)
    return waypoint_to_light

# test data:
lights_to_waypoints = [1, 3, 7, 8, 10, 15]
base_waypoints_num = 17

y = waypoint_to_light_f(lights_to_waypoints, base_waypoints_num)
# expected outcome:
x = (y == {0: (0, 1), 1: (1, 3), 2: (1, 3), 3: (2, 7), 4: (2, 7), 5: (2, 7), 6: (2, 7), 7: (3, 8), 8: (4, 10), 8: (4, 10),
                     9: (4, 10), 10: (5, 15), 11: (5, 15), 12: (5, 15), 13: (5, 15), 14: (5, 15), 15: (None, None), 16: (None, None)})

STATE_COUNT_THRESHOLD = 2 # 3 change to be smaller, as the frequency of processing camara image has reduced from about 10 Hz 3 Hz

class TLDetector(WaypointTracker):
    def __init__(self):
        WaypointTracker.__init__(self)

        rospy.init_node('tl_detector')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoint_to_light = None
        self.state_count = 0
        self.loop_freq = 2
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_color_cb)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # self.previous_traffic_light_position = 0
        # self.number_traffic_lights_passed = 0

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf_ros.TransformListener()
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        #rospy.spin()
        self.loop()

    def base_waypoints_cb(self, msg):
        WaypointTracker.base_waypoints_process(self, msg)
        # assumption that a traffic light can only have one waypoint close to it.
        # or one waypoint can have at most one traffic light near it.
        
        # implementation:
        # given a list of coordinates of traffic lights
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        light_cursor = 0
        base_waypoint_search_cursor = 0
        
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        lights_to_waypoints = []
        
        for light_cursor in range(len(stop_line_positions)):
            # take, l, the first of the remaining traffic lights coordinates list, self.stop_line_positions
            if base_waypoint_search_cursor < self.base_waypoints_num:
                dist_shortest = dl(self.base_waypoints[base_waypoint_search_cursor].pose.pose.position,
                                    stop_line_positions[light_cursor])
                light_waypoint_index = base_waypoint_search_cursor
        
                # for l to find the closest waypoint in the remaining base_waypoints, w
                for i in range(base_waypoint_search_cursor+1, self.base_waypoints_num):
                    dist = dl(self.base_waypoints[i].pose.pose.position,
                              stop_line_positions[light_cursor])
                    if dist < dist_shortest:
                        dist_shortest = dist
                        light_waypoint_index = i
                    # end of if dist < d_shortest
                # end of for i in range(base_waypoint_search_cursor+1, self.base_waypoints_num)
                # record the mapping from l to w
                lights_to_waypoints.append(light_waypoint_index)
                # remove l from the list of traffic lights, and w from the base_points
                base_waypoint_search_cursor = light_waypoint_index + 1
            else:
                lights_to_waypoints.append(None)
            # end of if base_waypoint_search_cursor < self.base_waypoints_num
        # end of for light_cursor in range(len(self.stop_line_positions))
        # until there is no more traffic light, or no more waypoint
        
        # construct the map, self.waypoint_to_light, the map from waypoint index to the index of the
        # traffic light in terms of the closest waypoint index
        self.waypoint_to_light = waypoint_to_light_f(lights_to_waypoints, self.base_waypoints_num)
        # rospy.loginfo('test using self.waypoint_to_light[237]: %r' % self.waypoint_to_light[237])
    def current_pose_cb(self, msg):
        self.pose = msg
    def traffic_array_cb(self, msg):
        self.lights = msg.lights
    
    def image_color_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
                of the waypoint closest to the red light's stop line to /traffic_waypoint
    
            Args:
                msg (Image): image from car-mounted camera
    
        """
        self.has_image = True
        self.camera_image = msg
    def get_light_state(self, light_index):
        """Determines the current color of the traffic light
    
        Args:
            light_index (TrafficLight): light to classify
    
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
    
        """
        FAKED_LIGHT = False
        if FAKED_LIGHT:
            rospy.loginfo('light_index: %d; state: %d; the light is RED: %r' % (
                light_index, self.lights[light_index].state,
                self.lights[light_index].state == TrafficLight.RED))
            return self.lights[light_index].state
        # end of if FAKED_LIGHT
    
        if(not self.has_image):
            self.prev_light_loc = None
            return None
    
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
        # self.stop_line_positions = self.config['stop_line_positions']
    
        if (self.base_waypoints and self.waypoint_to_light and self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #TODO find the closest visible traffic light (if one exists)
            # the index of the waypoint of the traffic light
            light_index, light_wp = self.waypoint_to_light[car_position]
            # self.find_closest_traffic_light(car_position)
            rospy.loginfo('light_index: %r; light waypoint: %r' % (light_index, light_wp))
            if light_index:
                state = self.get_light_state(light_index)
                return light_wp, state
            # end of if light_index
        # end of if (self.pose)
        return None, None
    def loop(self):
        rate = rospy.Rate(self.loop_freq)
        while not rospy.is_shutdown():
            if self.camera_image:
                light_wp, state = self.process_traffic_lights()
                if light_wp and state:
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
                # end of if light_wp and state
            # end of if self.camera_image
            self.camera_image = None
            rate.sleep()
        # end of while not rospy.is_shutdow()

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
