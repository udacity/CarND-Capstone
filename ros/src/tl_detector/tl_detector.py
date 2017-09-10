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
from traffic_light_config import config
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_pose = None
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

        # mockup tl_dection for testing, reading groud-truth directy from `/vehicle/traffic_lights`
        # TODO: use image classification for traffic light detection
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        

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
        self.car_pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

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

    def distance(self, waypoint, car_pose):
        wp_x, wp_y, wp_z = self.get_waypoint_coordinates(waypoint)
        car_x, car_y, car_z = self.get_car_coordinates(car_pose)

        dx = wp_x - car_x
        dy = wp_y - car_y
        dz = wp_z - car_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def inner_product(self, vec1, vec2):
        return sum([v1*v2 for v1, v2 in zip(vec1, vec2)])

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
        return 0


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

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

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # light = None
        # light_positions = config.light_positions
        # if(self.pose):
        #     car_position = self.get_closest_waypoint(self.pose.pose)

        # #TODO find the closest visible traffic light (if one exists)

        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # self.waypoints = None
        # return -1, TrafficLight.UNKNOWN


        traffic_light_indices = [290, 758, 2015, 2542, 6366, 7065, 8647, 9845]

        ahead_light = None
        ahead_light_dist = float('inf')
        if self.car_pose:
            for i, light in enumerate(self.lights):
                light_pose = light.pose.pose
                light_state = light.state
                if light_state != TrafficLight.RED: continue # ignore non-red lights
                light_wp = self.waypoints[traffic_light_indices[i]]
                if self.ahead_of(light_wp, self.car_pose):
                    d = self.distance(light_wp, self.car_pose)
                    if d < ahead_light_dist:
                        ahead_light = traffic_light_indices[i]
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
