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
        self.loop_freq = rospy.get_param('~loop_freq', 2)
        self.car_position = None        # the waypoint index in the base_waypoints of the waypoint in front of the car
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
        
        # self.previous_traffic_light_position = 0
        # self.number_traffic_lights_passed = 0

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
        # self.light_classifier = TLClassifier()
        use_simulator_classifier = rospy.get_param('~traffic_light_classifier_sim')
        self.light_classifier = TLClassifier(sim = use_simulator_classifier)
        
        self.listener = tf_ros.TransformListener()
        
        self.state = TrafficLight.UNKNOWN
        # self.last_state = TrafficLight.UNKNOWN
        self.last_wp = None
        self.state_count = 0
        
        #rospy.spin()
        self.loop()

    def base_waypoints_cb(self, msg):
        # copy the base_waypoints, compute the distance from the start to each base_waypoint,
        # to be able to compute distance among any two base_waypoints.
        WaypointTracker.base_waypoints_process(self, msg)
    
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
        # FAKED_LIGHT = False
        # if FAKED_LIGHT:
        #     rospy.loginfo('light_index: %d; state: %d; the light is RED: %r' % (
        #         light_index, self.lights[light_index].state,
        #         self.lights[light_index].state == TrafficLight.RED))
        #     return self.lights[light_index].state
        # end of if FAKED_LIGHT
    
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return None
    
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
    
        if ((self.base_waypoints is not None) and (self.waypoint_to_light is not None) and (self.pose is not None)):
            self.car_position = self.get_closest_waypoint(self.pose.pose)
            #TODO find the closest visible traffic light (if one exists)
            # the index of the waypoint of the traffic light
            light_index, light_wp = self.waypoint_to_light[self.car_position]
            FAKED_LIGHT = rospy.get_param('~use_simulator_light_state', False)
            # when the light_index is None, then is no more light in front
            if light_index is not None:
                if FAKED_LIGHT:
                    # rospy.loginfo('light_index: %d; state: %d; the light is RED: %r' % (
                    #     light_index, self.lights[light_index].state,
                    #     self.lights[light_index].state == TrafficLight.RED))
                    state = self.lights[light_index].state
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
    
                    #Get classification
                    state = self.light_classifier.get_classification(cv_image)
    
                # end of if FAKED_LIGHT
            else:
                state = TrafficLight.UNKNOWN
            # end of if light_index is not None
            if (state==TrafficLight.RED):
                rospy.loginfo('car index: %r; light_index: %r; light waypoint: %r; light is RED: %r' %
                              (self.car_position, light_index, light_wp, state==TrafficLight.RED))
            # end of if (state==TrafficLight.RED)
    
            return light_wp, state
        # end of if (self.pose)
        return None, TrafficLight.UNKNOWN
    def loop(self):
        rate = rospy.Rate(self.loop_freq)
        while not rospy.is_shutdown():
            if self.camera_image is not None:
                light_wp, state = self.process_traffic_lights()
                # only consider the traffic image when the car is close enough to the traffic light, say 20 waypoints
                if ((light_wp is not None) and (light_wp - self.car_position) < 100): # and state:
                    # Note: state might have value 0 and light_wp and 0 == False!
                    '''
                    Publish upcoming red lights at camera frequency.
                    Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                    of times till we start using it. Otherwise the previous stable state is
                    used.
                    '''
                    # rospy.loginfo('light_wp %d; state: %r, self.state: %r' % (light_wp, state, self.state))
                    # (abs(self.last_wp) != light_wp if self.last_wp else True) or
                    if (self.state is None) or (self.state != state):  # state changed
                        rospy.loginfo('state changed: old state count: %r; old state: %r; new state: %d' %
                        (self.state_count, self.state, state))
                        self.state_count = 0
                        self.state = state
                        # self.last_state = self.state
                        # self.last_wp = light_wp if (state == TrafficLight.RED) else -light_wp
                    elif self.state_count >= STATE_COUNT_THRESHOLD:
                        self.last_wp = light_wp if (state == TrafficLight.RED) else -light_wp
                        self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                        rospy.loginfo('stable state threshold reached: state count: %d; old state: %d; new state: %d; new traffic_waypoint: %d' %
                                    (self.state_count, self.state, state, self.last_wp))
                    else:
                        if self.last_wp is not None:
                            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                        # end of if self.last_wp is not None
                        rospy.loginfo('not enough state change: old state: %r; keep publish the old traffic_waypoint: %r' % (self.state, self.last_wp))
                    # end of if self.state != state
                    self.state_count += 1
                # end of if (light_wp is not None) and state
                self.camera_image = None
            # end of if self.camera_image is not None
            rate.sleep()
        # end of while not rospy.is_shutdow()

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
