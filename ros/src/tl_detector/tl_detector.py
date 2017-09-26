#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from route_traffic_lights import RouteTrafficLights, RouteTrafficLight
import tf
import cv2
import yaml
import math

USE_GROUND_TRUTH = True # use state from traffic lights topic
GT_PUBLISH_DISTANCE = 200 # min distance (in waypoints) when to start publishing gt traffic light data
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.use_ground_truth = USE_GROUND_TRUTH

        self.pose = None
        self.waypoints = None
        self.camera_image = None

        sub0 = rospy.Subscriber('/next_waypoint_ahead', Int32, self.wp_cb)
        #sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic
        light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available.
        You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.traffic_lights = RouteTrafficLights()

        self.next_waypoint_ahead = None 

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def wp_cb(self, msg):
        self.next_waypoint_ahead = msg.data

    #def pose_cb(self, msg):
    #    self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = list(waypoints.waypoints)
        # reset traffic light information on each route update
        self.traffic_lights.reset(self.waypoints, self.config['stop_line_positions'])

    def traffic_cb(self, msg):
        self.traffic_lights.update_states(msg.lights)

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state_gt(self, light_key):
        """
            Get traffic light status from ground truth messages
        """
        d = lambda x,y,s: (x - y + s) % s # distance in waypoints in a circular track
        
        stop_point = self.traffic_lights[light_key].stop_point

        if d(stop_point, self.next_waypoint_ahead, len(self.waypoints)) < GT_PUBLISH_DISTANCE:
            state = self.traffic_lights[light_key].state
        else:
            state = TrafficLight.UNKNOWN

        return state

    def get_light_state_from_image(self, light_key):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # TODO no point to process image if we are too far away

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(self.traffic_lights[light_key].pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """
        Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line i
                for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_wp = -1
        light_state = TrafficLight.UNKNOWN
        light_key = None

        if self.next_waypoint_ahead:
            light_wp, light_key = \
                self.traffic_lights.get_next_en_route(self.next_waypoint_ahead)

        if light_key:
            light_state = self.get_light_state(light_key) \
                if self.use_ground_truth == False \
                else self.get_light_state_gt(light_key)

        return light_wp, light_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
