#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose

from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

import waypoint_helper
import tf
import cv2
import yaml
import threading

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.traffic_lights_pos = waypoint_helper.get_traffic_lights()

        # Need to set the server parameter for site or test
        self.run_mode = rospy.get_param("/run_mode", "site")
        self.tl_classifier = TLClassifier(self.run_mode)
        print("Traffic light classifier created")

        self.listener = tf.TransformListener()
        print("Listening")

        # we need to wait till system is set up otherwise we ge race condition
        # but don't lock the main thread

        self.classifier_ready = False

        def shutdown_timer():
            self.classifier_ready = True
            timer.cancel()

        timer = threading.Timer(15.0, shutdown_timer())
        timer.daemon = True
        timer.start()

        # Subscribers
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


        self.bridge = CvBridge()

        # Publishers
        # For publishing the waypoint index of a red traffic light
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        # For publishing the upcomming traffic light state
        self.upcoming_traffic_light_state_pub = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)
        # For publishing the upcomming traffic light position
        # Note: no information on the z
        self.upcoming_traffic_light_pos_pub = rospy.Publisher('/traffic_light_pos', geometry_msgs.msg.Point, queue_size=1)

        self.tl_detection_out = rospy.Publisher('/tl_detection_out', Image, queue_size=1)

        self.lock = threading.RLock()
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

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
        # Need to protect agains race condition - synchronous cb
        if self.lock.aquire(True):

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

        self.lock.release()

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
        if self.classifier_ready is True:
            return self.tl_classifier.get_classification(cv_image)

        return None

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image or not self.pose or not self.waypoints or not self.traffic_lights_pos):
            return False

        state = TrafficLight.UNKNOWN   # Default state

        closest_traffic_light_idx, closest_traffic_light = waypoint_helper.get_closest_traffic_light(
                                self.traffic_positions.lights, self.pose.position, self.waypoints)

        if closest_traffic_light:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

            if self.classifier_ready is True:
                state = self.tl_classifier.get_classification(cv_image)

                self.tl_detection_out.publish(self.bridge.cv2_to_imgmsg(
                    self.tl_classifier.detection_image, "rgb8"))


            # Publish state
            self.upcoming_traffic_light_state_pub.publish(Int32(state))
            # Publish pos
            self.upcoming_traffic_light_pos_pub.publish(closest_traffic_light.pose.pose.position)

            return closest_traffic_light_idx, state

        self.waypoints = None

        return -1, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
