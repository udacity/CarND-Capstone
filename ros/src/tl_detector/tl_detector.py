#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from math import sqrt

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        # self.pub_ready = rospy.Publisher('tl_detector_ready', Bool)
        rospy.init_node('tl_detector')
        rospy.logdebug('Init TL DETECTOR')
        # self.pub_ready.publish(False)
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

        # /image_color subscriber
        self.sub_image = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        rospy.logwarn('------ classifier loaded -------')
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.image_index = 0

        self.has_image = False

        self.loop()

    def loop(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():

            light, should_evaluate, debug_state = self.get_closest_light()
            if should_evaluate and self.sub_image is None:
                self.sub_image = rospy.Subscriber('/image_color', Image, self.image_cb)
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
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

        self.sub_image.unregister()
        self.sub_image = None

        light_wp, state = self.process_traffic_lights()
        #rospy.loginfo(state) #
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
            rospy.loginfo('light_wp %d', light_wp) #
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def save_image(self, state):
        if hasattr(self.camera_image, 'encoding'):
            self.attribute = self.camera_image.encoding
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv2.imwrite('/tmp/img%04d_%d.png'%(self.image_index, state),cv_image)
        self.image_index += 1

    def pose_distance(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return sqrt(dx**2+dy**2)

    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_dist = float("inf")
        closest_index = None

        for i, waypoint in enumerate(waypoints):
            d = self.pose_distance(pose, waypoint.pose.pose)
            if (d<closest_dist):
                closest_dist = d
                closest_index = i

        return closest_index

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
        return self.light_classifier.get_classification(cv_image)

    def get_closest_light(self):
        """Finds closest visible traffic light, if one exists, and determines its
           if it's close enough to evaluate the camera image

           Returns:
             waypoint: light
             bool: is the light close enough to evaluate camera image?
             int: state of light

        """
        light = None
        should_evaluate = False
        state = TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose is None or self.waypoints is None:
            return None, False, TrafficLight.UNKNOWN

        car_position = self.get_closest_waypoint(self.pose.pose, self.waypoints)

        closest_light_index = self.get_closest_waypoint(self.pose.pose, self.lights)
        if closest_light_index is None:
            return None, False, TrafficLight.UNKNOWN

        rospy.loginfo('closest_light_index: %d', closest_light_index) #
        #rospy.loginfo(self.lights)


        if closest_light_index is not None:
            closest_light = self.lights[closest_light_index]
            #rospy.loginfo(closest_light)
            closest_light_state = closest_light.state
            #self.save_image(closest_light_state)
            light = closest_light

        if light:
            light_wp = self.get_closest_waypoint(light.pose.pose, self.waypoints)

            light_prediction_waypoints = 120
            waypoints_to_next_light_waypoint = light_wp - car_position
            if light_wp > (len(self.waypoints) - light_prediction_waypoints) <= light_prediction_waypoints:
                waypoints_to_next_light_waypoint += len(self.waypoints)
            rospy.loginfo('waypoint distance is %s', waypoints_to_next_light_waypoint)
            if waypoints_to_next_light_waypoint >= 25 and waypoints_to_next_light_waypoint <= light_prediction_waypoints:
                should_evaluate = True
                rospy.logwarn('should evaluate light')

        return light, should_evaluate, state



    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light, should_evaluate, debug_state = self.get_closest_light()

        if should_evaluate:
            state = self.get_light_state(light)
            if state != TrafficLight.UNKNOWN:
                rospy.loginfo('predicted state: %d, actual state: %d', state, light.state)
                # rospy.loginfo('car_wp: %d', car_position)

            light_wp = self.get_closest_waypoint(light.pose.pose, self.waypoints)

            light_pose = Pose()
            light_pose.position.x = light
            #self.pub_ready.publish(True)
            return light_wp, state

        # TODO: why are waypoints destroyed here?
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
