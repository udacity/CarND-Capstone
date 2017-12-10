#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from operator import itemgetter

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    dl = lambda self, a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    # Details from src/styx_msgs/msg/TrafficLight.msg
    state_txt = { 0: 'RED', 1: 'YELLOW', 2: 'GREEN', 4: 'UNKNOWN' }

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light_waypoints = []
        self.light_visible = False

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
        self.waypoints = None
        self.waypoints_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # /base_waypoints should only be passed once at initialisation
        if self.waypoints:
            rospy.logerr("/base_waypoints message received multiple times!")
        self.waypoints = waypoints.waypoints
        self.waypoints_count = len(self.waypoints)
        rospy.logwarn("{:,} waypoints received from /base_waypoints.".format(self.waypoints_count))

    def traffic_cb(self, msg):
        """
        Use the vehicle's location and the (x, y) coordinates for traffic light
            stop lines to find the nearest visible traffic light ahead of the vehicle.
        You will want to use the get_closest_waypoint method to find the closest
            waypoints to the vehicle and lights.
        Using these waypoint indices, you can determine which light is ahead of
            the vehicle along the list of waypoints.
        """
        self.lights = msg.lights
        if self.light_waypoints==[] and self.waypoints:
            # Create waypoints for the traffic signals and also the associated stop lines.
            self.light_waypoints = [self.get_closest_waypoint(light.pose.pose) for light in self.lights]
            rospy.logwarn("traffic light waypoints calculated as {}.".format(self.light_waypoints))
            self.stopline_waypoints = \
                [ self.get_closest_waypoint(Pose(Point(x,y,0.0),Quaternion(0.0,0.0,0.0,0.0))) for (x, y) in self.config['stop_line_positions'] ]
            rospy.logwarn("traffic light stopline waypoints calculated as {}.".format(self.stopline_waypoints))

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, line_wp, state = self.process_traffic_lights()

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
            if self.last_state != self.state:
                rospy.logwarn("traffic light: {}.".format(self.state_txt[self.state]))
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            line_wp = line_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(line_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Process if the waypoints list exists (from /base_waypoints)
        if self.waypoints:
            distances = [self.dl(pose.position, waypoint.pose.pose.position) for waypoint in self.waypoints]
            return min(enumerate(distances), key=itemgetter(1))[0]
        else:
            return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO: Temporary data from simulator until image processing is complete.
        return self.lights[light].state

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
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        lights = [waypoint for waypoint in self.light_waypoints if waypoint>=car_position and waypoint<=car_position+200]
        light_waypoint = None if len(lights)==0 else lights[0]

        if light_waypoint:
            if light_waypoint in self.light_waypoints:
                light = self.light_waypoints.index(light_waypoint)
                stopline_waypoint = self.stopline_waypoints[light]
                state = self.get_light_state(light)
                if not self.light_visible:
                    rospy.logwarn("traffic light approaching - {}.".format(self.state_txt[state]))
                    self.light_visible = True
                return light_waypoint, stopline_waypoint, state
        if self.light_visible:
            rospy.logwarn("traffic light passed.")
            self.light_visible = False
        return -1, -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
