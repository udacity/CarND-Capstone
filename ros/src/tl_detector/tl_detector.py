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
import math
import yaml
from attrdict import AttrDict
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.position_array = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size= STATE_COUNT_THRESHOLD)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size= STATE_COUNT_THRESHOLD)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size= STATE_COUNT_THRESHOLD)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = STATE_COUNT_THRESHOLD)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size= STATE_COUNT_THRESHOLD)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.position_array = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in self.waypoints]

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.loginfo("GROUND TRUTH Traffic Lights (%s)", msg.lights)

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
            light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
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
        #TODO_Done implement

        def dl(a, b):
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

        closest_wp = len(self.waypoints)-1

        wp_lower = 0
        wp_upper = len(self.waypoints) - 1
        while wp_lower < wp_upper:
            wp_mid = (wp_lower + wp_upper) // 2
            dist_lower = dl(self.waypoints[wp_lower].pose.pose.position, pose)
            dist_upper = dl(self.waypoints[wp_upper].pose.pose.position, pose)
            dist_mid = dl(self.waypoints[wp_mid].pose.pose.position, pose)

            closest_dist = dist_lower
            closest_wp = wp_lower
            if dist_mid < closest_dist:
                closest_dist = dist_mid
                closest_wp = wp_mid
            if dist_upper < closest_dist:
                closest_dist = dist_upper
                closest_wp = wp_upper


            # if all contiguous its converged wp_lower contains the closest_wp
            if wp_lower == wp_mid -1 and wp_mid == wp_upper -1:
                break

            # handle track looping around to start
            if dist_upper < dist_mid:
                if dist_lower < dist_upper:
                    wp_upper = wp_mid - 1
                else:
                    wp_lower = wp_mid + 1

            elif dist_mid < closest_dist:
                wp_lower = wp_mid-1
            elif closest_dist < dist_mid:
                wp_upper = wp_mid+1
            elif dist_lower < dist_upper:
                wp_upper = wp_mid + (wp_upper - wp_mid) // 2
            elif dist_upper < dist_lower:
                wp_lower = wp_mid - (wp_mid - wp_lower) // 2

        return closest_wp



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
        rospy.logdebug ("image shape (%s)", np.shape(cv_image))
        #cv2.imwrite('tl.png', cv_image)

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO_not required use light location to zoom in on traffic light in image

        #_cv2.imwrite('output_images/tlroi', cv_image) //roi image here


        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if (self.waypoints is None):
            rospy.logwarn("Not processing traffic lights as waypoints are not initialized")
            return -1, TrafficLight.UNKNOWN

        light = None
        car_position = -1
        light_wp = -1
        nearest_light_car_index = 0

        # List of positions that correspond to the line to stop in front of for a given intersection
        light_positions = self.config['stop_line_positions']
        rospy.logdebug("light_positions (%s)", light_positions)

        if(self.pose):
            #car_position = self.get_closest_waypoint(AttrDict({'x': self.pose.pose.position.x, 'y': self.pose.pose.position.y}))
            pose_vector = np.array([self.pose.pose.position.x, self.pose.pose.position.y])
            nearest_light_car_index = np.array([np.linalg.norm(x+y) for (x,y) in light_positions-pose_vector]).argmin()
            rospy.logdebug("closest light to car position (%s) is at (%s)", self.pose.pose.position, light_positions[nearest_light_car_index])


        #TODO_Done find the closest visible traffic light (if one exists)
        light_position = light_positions[nearest_light_car_index]
        tmp_light_wp = self.get_closest_waypoint(AttrDict({'x': light_position[0], 'y': light_position[1]}))

        # Check if waypoint nearest to traffic light is ahead of the car
        if  self.waypoints[tmp_light_wp].pose.pose.position.x >= self.pose.pose.position.x :
            light_wp = tmp_light_wp
            light = self.lights[nearest_light_car_index]

            rospy.logdebug("Upcoming closest light to vehicle's position (%s, %s) is nearest to waypoint index  (%s) and is "
                      "at location (%s, %s, %s)",
                          self.pose.pose.position.x, self.pose.pose.position.y,
                          light_wp, light.pose.pose.position.x, light.pose.pose.position.y, light.pose.pose.position.z)
            rospy.logdebug("Light's nearest Waypoint (%s) Details (%s)",  light_wp, self.waypoints[light_wp].pose.pose.position)

            rospy.loginfo("GROUND TRUTH Light state near waypoint (%s) is (%s)", light_wp, light.state)


        if light:
            # Uncomment below line to test waypoint publishing
            # Comment out below before final submission
            #return light_wp, light.state

            state = self.get_light_state(light)
            rospy.loginfo("CLASSIFIER Light state near waypoint (%s) is (%s)", light_wp, state)

            return light_wp, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
