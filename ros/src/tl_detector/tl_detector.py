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
        def distance_to_waypoint(pose, waypoint):
            veh_x = pose.position.x
            veh_y = pose.position.y
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dx = veh_x - wp_x
            dy = veh_y - wp_y
            return math.sqrt(dx * dx + dy * dy)

        nearest = None
        min_dist = float("inf")
        for index, waypoint in enumerate(self.waypoints.waypoints):
            dist = distance_to_waypoint(pose, waypoint)
            if dist < min_dist:
                min_dist = dist
                nearest = index

        return nearest


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
        # TODO(Olala): return the real state
        # Fake light detection with ground truth
        if not self.lights:
            return TrafficLight.UNKNOWN
        light_index = self.get_closest_waypoint(light.pose)
        for light_3d in self.lights:
            light_3d_index = self.get_closest_waypoint(light_3d.pose.pose)
            if abs(light_index - light_3d_index) < 50:
                return light_3d.state
        return TrafficLight.UNKNOWN
        # End of fake light detection

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.waypoints or not self.pose:
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # car_fwd indicates whether car's moving the same direction as the waypoint index increase
        car_fwd = False
        closest_wp_index = None
        waypoints = self.waypoints.waypoints
        wp_length = len(waypoints)
        if(self.pose):
            car_orientation = self.pose.pose.orientation
            quaternion = (car_orientation.x, car_orientation.y,
                          car_orientation.z, car_orientation.w)
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

            car_pose = self.pose.pose
            closest_wp_index = self.get_closest_waypoint(car_pose)
            wp1 = waypoints[closest_wp_index]
            wp2 = waypoints[closest_wp_index + 1 % wp_length]
            fwd_angle = math.atan2(wp2.pose.pose.position.y - wp1.pose.pose.position.y,
                                   wp2.pose.pose.position.x - wp1.pose.pose.position.x)

            if -math.pi / 2.0 < yaw - fwd_angle < math.pi / 2:
                car_fwd = True

        #TODO find the closest visible traffic light (if one exists)
        traffic_lights = dict()
        for stop_line in stop_line_positions:
            stop_line_pose = Pose()
            stop_line_pose.position.x = stop_line[0]
            stop_line_pose.position.y = stop_line[1]
            line_position = self.get_closest_waypoint(stop_line_pose)
            traffic_lights[line_position] = stop_line_pose

        light, light_wp = None, None

        for i in range(wp_length):
            inc = 1 if car_fwd else -1
            index = (closest_wp_index + i * inc) % wp_length
            if index in traffic_lights:
                light = TrafficLight()
                light_wp = index
                light.pose = traffic_lights[index]
                break

        if light:
            state = self.get_light_state(light)
            rospy.loginfo('light color ahead %s' % state)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
