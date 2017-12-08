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

def color_code_to_label(color_code):
      if color_code == TrafficLight.GREEN:
          color_label = "GREEN"
      elif color_code == TrafficLight.RED:
          color_label = "RED"
      elif color_code == TrafficLight.YELLOW:
          color_label = "YELLOW"
      else:
          color_label = "UNKNOWN"
      # end of if color_code == TrafficLight.GREEN
      return color_label

STATE_COUNT_THRESHOLD = 0
# It works with value zero, effectively an event will be considered stable
# the second time received the same.
# Be careful, it cannot be too large to avoid lose of event being reported.

class TLDetector(WaypointTracker):
    def __init__(self):
        WaypointTracker.__init__(self)

        rospy.init_node('tl_detector')

        self.camera_image = None
        self.lights = []
        self.loop_freq = rospy.get_param('~loop_freq', 2)
        self.car_index = None
        # the waypoint index in the base_waypoints of the waypoint in front of the car
        
        self.FAKED_LIGHT = rospy.get_param('~use_simulator_light_state', False)
        self.admissible_distance_for_image = rospy.get_param('~admissible_distance_for_image', 80)

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

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
        use_simulator_classifier = rospy.get_param('~traffic_light_classifier_sim')
        self.light_classifier = TLClassifier(sim = use_simulator_classifier)
        
        self.listener = tf_ros.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_wp = None
        self.state_count = 0

        self.loop()

    def base_waypoints_cb(self, msg):
        # copy the base_waypoints, compute the distance from the start to each base_waypoint,
        # to be able to compute distance among any two base_waypoints.
        WaypointTracker.base_waypoints_process(self, msg)

    def preprocess(self):
        if self.base_waypoints:
            WaypointTracker.preprocess(self)
            self.ready = True

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

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
    
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
    
        """
        light = None
    
        if ((self.base_waypoints is not None) and
            (self.waypoint_to_light is not None) and
            (self.pose is not None)):
            self.car_index = self.get_closest_waypoint(self.pose.pose)
            # DONE find the closest visible traffic light (if one exists)
            # the index of the waypoint of the traffic light
            light_index, light_wp = self.waypoint_to_light[self.car_index]
            if light_wp is None:
                return light_wp, TrafficLight.UNKNOWN
            # end of if light_wp is None
    
            if (self.admissible_distance_for_image < self.distance(self.car_index, light_wp)):  # beyond 150 meters
                return light_wp, TrafficLight.UNKNOWN
            else:
            # when the light_index is None, then is no more light in front
                if light_index is not None:
                    if self.FAKED_LIGHT:
                        # rospy.loginfo('light_index: %d; state: %d; the light is RED: %r' % (
                        #     light_index, self.lights[light_index].state,
                        #     self.lights[light_index].state == TrafficLight.RED))
                        state = self.lights[light_index].state
                    else:
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
    
                        # Get classification
                        state = self.light_classifier.get_classification(cv_image)
    
                    # end of if self.FAKED_LIGHT
                else:
                    state = TrafficLight.UNKNOWN
                # end of if light_index is not None
                # if (state==TrafficLight.RED):
                #     rospy.loginfo('car index: %r; light_index: %r; light waypoint: %r; light is RED: %r' %
                #                   (self.car_index, light_index, light_wp, state==TrafficLight.RED))
                # end of if (state==TrafficLight.RED)
    
            # end of if (self.admissible_distance_for_image < self.distance(self.car_index, light_wp))
            return light_wp, state
        # end of if ((self.base_waypoints is not None) and
            # (self.waypoint_to_light is not None) and
            # (self.pose is not None))
        return None, TrafficLight.UNKNOWN

    def loop(self):
        rate = rospy.Rate(self.loop_freq)
        while not rospy.is_shutdown():
            if not self.ready:
                self.preprocess()
            else:
                if self.camera_image is not None:
                    light_wp, state = self.process_traffic_lights()
                    '''
                        Publish upcoming red lights at camera frequency.
                        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                        of times till we start using it. Otherwise the previous stable state is
                        used.
                        '''
                    # rospy.loginfo('light_wp %d; state: %r, self.state: %r' % (light_wp, state, self.state))
                    if (self.state is None) or (self.state != state):  # state changed
                        # rospy.loginfo('state changed: old state count: %r; old state: %r; new state: %d; light_waypoint: %r' %
                        #               (self.state_count, self.state, state, light_wp))
                        rospy.loginfo("from {:7} to {:7} state counter {:3} light at {:7}; changed: state or traffic light index".format(
                            color_code_to_label(self.state), color_code_to_label(state), self.state_count, light_wp))
    
                        self.state_count = 0
                        self.state = state
                    elif (self.state_count >= STATE_COUNT_THRESHOLD) and light_wp is not None:
                        if (state != TrafficLight.UNKNOWN):
                            # rospy.loginfo(
                            #     'stable state threshold reached: state count: %d; old state: %d; new state: %d; new traffic_waypoint: %r' %
                            #     (self.state_count, self.state, state, self.last_wp))
                            self.last_wp = light_wp if (state == TrafficLight.RED) else -light_wp
                            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                            rospy.loginfo("from {:7} to {:7} state counter {:3} light at {:7}; stable; reporting {}".format(
                                color_code_to_label(self.state), color_code_to_label(state), self.state_count, light_wp, self.last_wp))
    
                        # end of if (state == TrafficLight.RED)
                    else:
                        if self.last_wp is not None:
                            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                        # end of if self.last_wp is not None
                        # rospy.loginfo('not enough state change: old state: %r; keep publish the old traffic_waypoint: %r' % (self.state, self.last_wp))
                        rospy.loginfo("from {:7} to {:7} state counter {:3} light at {:7}; not yet stable: reporting last state and light traffic index, reporting {}".format(
                            color_code_to_label(self.state), color_code_to_label(state), self.state_count, light_wp, self.last_wp))
    
                    # end of if (self.state is None) or (self.state != state)
                    self.state_count += 1
                    self.camera_image = None
                # end of if self.camera_image is not None
            rate.sleep()
        # end of while not rospy.is_shutdow()

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
