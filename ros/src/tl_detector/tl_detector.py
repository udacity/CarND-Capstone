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
USE_GROUND_TRUTH = False
LIGHT_SCAN_DISTANCE = 100

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []
        self.state = None

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

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        if self.waypoints == None:
            return -1
        
        x = pose.position.x
        y = pose.position.y

        point_index = None
        min_distance = float("inf")
        for i, waypoint in enumerate(self.waypoints.waypoints):
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            dist = math.sqrt(pow(x-waypoint_x,2)+pow(y-waypoint_y,2))
            if dist < min_distance:
                min_distance = dist
                point_index = i

        return point_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if USE_GROUND_TRUTH:
            if self.lights == None:
                return TrafficLight.UNKNOWN
            state = self.lights[light].state
            return state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        if(self.camera_image.encoding == "rgb8"):
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.pose == None:
            return -1, TrafficLight.UNKNOWN
        
        if self.waypoints == None:
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        car_x = self.pose.pose.position.x
        car_y = self.pose.pose.position.y

        red_lights = []
        for i, l in enumerate(self.lights):
            x = l.pose.pose.position.x
            y = l.pose.pose.position.y
            distance = math.sqrt((car_x-x)**2 + (car_y-y)**2)
            if distance > LIGHT_SCAN_DISTANCE:
                continue
            waypoint = self.nearest_waypoint(x, y)
            if waypoint > car_position:
                light = i
                light_wp = self.nearest_waypoint(stop_line_positions[i][0], stop_line_positions[i][1])
                break

        if light != None:
            state = self.get_light_state(light)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN

    def nearest_waypoint(self, x, y):
        min_distance = float("inf")
        min_distance_waypoint = 0
        for i in range(0, len(self.waypoints.waypoints)):
            waypoint = self.waypoints.waypoints[i]
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            distance = math.sqrt((x-waypoint_x)**2 + (y-waypoint_y)**2)
            if distance < min_distance:
                min_distance = distance
                min_distance_waypoint = i
        return min_distance_waypoint

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
