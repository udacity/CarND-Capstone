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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        # mockup tl_dection for testing, reading groud-truth directy from `/vehicle/traffic_lights`
        # TODO: use image classification for traffic light detection
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        # sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub6 = rospy.Subscriber(rospy.get_param("~camera_topic"), Image, self.image_cb)
        

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        
        # implemented as having the same frequency as `/image_color`
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.scenario = rospy.get_param("~scenario")
        self.light_classifier = TLClassifier(self.scenario)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        # self.counter = 0

        rospy.spin()

    ############################### subscriber callbacks #################################

    def pose_cb(self, msg):
        self.car_pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        # receive once
        self.base_waypoints_sub.unregister()

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


    ################################## utility functions #################################

    def ahead_of(self, waypoint, car_pose):
        """If a waypoint is ahead of the car based on its current pose.
        Logic: In the local coordinate system (car as origin), the angel
        between the waypoint vector and the car's current yaw vector should be
        less than 90, which also means their innter product should be positive.
        """
        wp_x, wp_y, wp_z = self.get_waypoint_coordinates(waypoint)
        car_x, car_y, car_z = self.get_car_coordinates(car_pose)
        _, _, car_yaw = self.get_car_euler(car_pose)

        wp_vector = (wp_x-car_x, wp_y-car_y)
        yaw_vector = (math.cos(car_yaw), math.sin(car_yaw))

        return self.inner_product(wp_vector, yaw_vector) > 0

    def get_car_euler(self, car_pose):
        """Return roll, pitch, yaw from car's pose
        """
        return tf.transformations.euler_from_quaternion([
            car_pose.orientation.x,
            car_pose.orientation.y,
            car_pose.orientation.z,
            car_pose.orientation.w])

    def get_car_coordinates(self, car_pose):
        car_x = car_pose.position.x
        car_y = car_pose.position.y
        car_z = car_pose.position.z
        return (car_x, car_y, car_z)

    def get_waypoint_coordinates(self, waypoint):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        return (wp_x, wp_y, wp_z)

    def get_light_coordinates(self, light):
        x = light.pose.pose.position.x
        y = light.pose.pose.position.y
        z = light.pose.pose.position.z
        return (x, y, z)

    def distance(self, xyz1, xyz2):
        x1, y1, z1 = xyz1
        x2, y2, z2 = xyz2
        dx, dy, dz = x1-x2, y1-y2, z1-z2
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def inner_product(self, vec1, vec2):
        return sum([v1*v2 for v1, v2 in zip(vec1, vec2)])

    def get_closest_waypoint_index(self, light):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            light: light position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        Assumes self.waypoints is already available

        """
        distances = [self.distance(self.get_light_coordinates(light),
                                   self.get_waypoint_coordinates(wp)) 
                    for wp in self.waypoints]
        return distances.index(min(distances))



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

    #######################  light state detection #######################################

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


        x, y = self.project_to_image_plane(light.pose.pose.position)

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        ahead_light = None
        ahead_light_dist = float('inf')
        if self.car_pose:
            for i, light in enumerate(self.lights):
                if self.distance(self.get_light_coordinates(light), 
                                 self.get_car_coordinates(self.car_pose)) >= 90:
                    continue

                light_state = self.get_light_state(light)
                if light_state != TrafficLight.RED: continue # ignore non-red lights
                
                light_wp_index = self.get_closest_waypoint_index(light)
                light_wp = self.waypoints[light_wp_index]
                if self.ahead_of(light_wp, self.car_pose):

                    d = self.distance(self.get_waypoint_coordinates(light_wp),
                                      self.get_car_coordinates(self.car_pose))
                    
                    if d < ahead_light_dist:
                        ahead_light = light_wp_index
                        ahead_light_dist = d

        if ahead_light is not None:
            return ahead_light, TrafficLight.RED
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
