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
import numpy as np
import yaml

STATE_COUNT_THRESHOLD = 3
MAX_DIST              = 100.0
DEBUG                 = False

class Point:
    def __init__(self, t):
        self.x = t[0]
        self.y = t[1]

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
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_len = 100000
        closest_wp_i = 0

        if self.waypoints is not None:
            waypoints = self.waypoints
            dl = lambda a, b: (a.x - b.x) ** 2 + (a.y - b.y) ** 2
            for i in range(len(waypoints)):
                dist = dl(pose, waypoints[i].pose.pose.position)
                if dist < closest_len:
                    closest_len = dist
                    closest_wp_i = i
        return closest_wp_i


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        x  = -1 
        y  = -1

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        rot   = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        if (rot is not None) and (trans is not None):

            #TODO Use tranform and rotation to calculate 2D position of light in image
            trans_mat = tf.transformations.translation_matrix(trans)
            rot_mat   = tf.transformations.quaternion_matrix(rot)

            M  = np.dot(trans_mat, rot_mat)
            point_hom = np.array([[point_in_world.x], [point_in_world.y], [point_in_world.z], [1.0]])
            point_veh = np.dot(M, point_hom)
            
            # Transform points to image plane
            x = -fx * point_veh[1]/point_veh[0] + 0.5*image_width
            y = -fy * point_veh[2]/point_veh[0] + 0.5*image_height


            if DEBUG:
              rospy.loginfo('trans: {}'.format(trans))
              rospy.loginfo('rot: {}'.format(rot))
              rospy.loginfo('point_x: {}, point_y: {} point_z: {}'.format(point_in_world.x, point_in_world.y, point_in_world.z))
              rospy.loginfo('point_x_veh: {}, point_y_veh: {} point_z_veh: {}'.format(point_veh[0], point_veh[1], point_veh[2]))
              rospy.loginfo('x_img: {}, y_img: {}'.format(x, y))

        return (x, y)

    def check_inside_image(self, x, y):
        inside = ( (x is not None) and (y is not None)  
                   and (x >= 0)         and (x <= self.config['camera_info']['image_width'])
                   and (y >= 0)         and (x <= self.config['camera_info']['image_height']) )

        return inside

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

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        image_width  = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        w_img = int(0.15*image_width)
        h_img = int(image_height/8)

        top    = int(image_height*0.05)
        bottom = int(y + h_img)
        left   = int(x - w_img)
        right  = int(x + w_img)

        tlState = TrafficLight.UNKNOWN
        if self.check_inside_image(left,top) and self.check_inside_image(bottom, right):
            roi = cv_image[top:bottom, left:right]
            #self.deb_img.publish(self.bridge.cv2_to_imgmsg(crop, "bgr8"))
            tlState = self.light_classifier.get_classification(roi)

        #Get classification
        if DEBUG:
            rospy.loginfo('tlState: {}'.format(tlState))

        return tlState


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = self.config['light_positions']
        light_distance_squared  = 10000*MAX_DIST


        if( (self.pose) and (len(self.lights) > 0)):
            car_position = self.get_closest_waypoint(self.pose.pose.position)

            #TODO find the closest visible traffic light (if one exists)
            first = True
                
            for i, light_pos in enumerate(light_positions):
                
                light_point    = Point(light_pos)
                light_waypoint = self.get_closest_waypoint(light_point)
                if first and light_waypoint >= car_position:
                    first = False
                    light_wp = light_waypoint
                    light = self.lights[i]
                    light_distance_squared = (light_point.x-self.pose.pose.position.x)**2 + (light_point.y-self.pose.pose.position.y)**2
                elif light_waypoint >= car_position and light_waypoint < light_wp:
                    light_wp = light_waypoint
                    light = self.lights[i]
                    light_distance_squared = (light_point.x-self.pose.pose.position.x)**2 + (light_point.y-self.pose.pose.position.y)**2
        

        if light:
            if DEBUG:
                rospy.loginfo('light_wp: {}'.format(light_wp))


            if (light_distance_squared >= MAX_DIST*MAX_DIST):
                return -1, TrafficLight.UNKNOWN
            else:
                state = self.get_light_state(light)
                return light_wp, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
