#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import os

STATE_COUNT_THRESHOLD = 3
SPIN_FREQUENCY = 30

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
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.spin(SPIN_FREQUENCY)

    def spin(self, freq):
        """
        Spins this ROS node based on the given frequency.

        :param freq: frequency in hertz.
        """
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.pose is not None and self.waypoints is not None and self.camera_image is not None:
                light_wp, state = self.process_traffic_lights()
                # once process traffic light set camera_image to None so if no image coming we will skip this block
                self.camera_image = None
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
            rate.sleep()

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
        self.has_image = True
        self.camera_image = msg

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        distance = float('inf')
        dist_wp = lambda w1, w2: math.sqrt((w1.x-w2.x)**2 + (w1.y-w2.y)**2  + (w1.z-w2.z)**2)

        index_wp = 0
        for i in range(len(self.waypoints.waypoints)):
            new_distance  = dist_wp (pose.position, self.waypoints.waypoints[i].pose.pose.position)
            if new_distance < distance:
                distance = new_distance
                index_wp = i

        return index_wp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Manual Testing
        return light.state

        # This block will be back with Classifier
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False
        #
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #
        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def create_light(self, x, y, z, yaw):
        light = TrafficLight()

        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z

        return light
		
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

            # find the closest visible traffic light (if one exists)
            distance = float('inf')
            for i, stop_line_position in enumerate(stop_line_positions):
                temp_light = self.create_light(stop_line_position[0], stop_line_position[1], 0.0, 0.0)
                temp_light.state = self.lights[i].state
                light_position = self.get_closest_waypoint(temp_light.pose.pose)
                car_x = self.waypoints.waypoints[car_position].pose.pose.position.x
                car_y = self.waypoints.waypoints[car_position].pose.pose.position.y
                light_x = self.waypoints.waypoints[light_position].pose.pose.position.x
                light_y = self.waypoints.waypoints[light_position].pose.pose.position.y
                temp_distance = self.distance(car_x, car_y, light_x,light_y)

                # use the light ahead of car
                if (light_position > car_position) and (temp_distance < distance):
                    light = temp_light
                    closest_light_wp = light_position
                    distance = temp_distance 

            if light:
                state = self.get_light_state(light)

                rospy.loginfo('Bugbug next traffic light {}, color: {}'.format(closest_light_wp, state))

                return closest_light_wp, state

        #bugbug this line will cause error between last light and end of lap
        #self.waypoints = None
        rospy.loginfo('Bugbug no next traffic light')
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
