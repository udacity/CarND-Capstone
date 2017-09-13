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
from train_queue import TrainQueue, TrainItem
import yaml

STATE_COUNT_THRESHOLD = 3

def equal_light(a, b):
    if not a.pose.pose.position == b.pose.pose.position:
        return False
    if not a.pose.pose.orientation == b.pose.pose.orientation:
        return False
    return True

def equal_lights(lights, other_lights):
    for light in lights:
        found = False
        for other in other_lights:
            if equal_light(light, other):
                found = True
                break
        if not found:
            return False
    return True

def euclidean_distance(p1x, p1y, p2x, p2y):
    x_dist = p1x - p2x
    y_dist = p1y - p2y
    return math.sqrt(x_dist*x_dist + y_dist*y_dist)

def find_closest_light(lights, pose):
    closest_light_distance = float("inf")
    closest_light = None
    for light in lights:
        light_position = light.pose.pose.position
        distance = euclidean_distance(light_position.x, light_position.y, pose.position.x, pose.position.y)
        if distance < closest_light_distance:
            closest_light_distance = distance
            closest_light = light
    return (closest_light, closest_light_distance)

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light_distance = None
        self.behind = None
        self.closest = None
        self.train_queue = TrainQueue()

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
        # print('ego', self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)

    def waypoints_cb(self, waypoints): # type: Lane
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        # if self.lights:
            # print('equal_lights', equal_lights(self.lights, msg.lights))
        self.lights = msg.lights
        # print('len', len(self.lights))
        closest, distance = find_closest_light(self.lights, self.pose.pose)

        if self.light_distance:
            if distance > self.light_distance:
                self.behind = True
            elif distance < self.light_distance:
                self.behind = False

        # print('closest', 's', closest.state, 'behind', self.behind, 'distance', distance, 'pos', closest.pose.pose.position.x, closest.pose.pose.position.y, closest.pose.pose.position.z, 'pose.pose.orient', closest.pose.pose.orientation.x, closest.pose.pose.orientation.y, closest.pose.pose.orientation.z, closest.pose.pose.orientation.w)
        # for light in self.lights:
        self.light_distance = distance
        self.closest = closest

    def friendly_name(self, state):
        state_name_dict = {TrafficLight.UNKNOWN: "Unknown",
                           TrafficLight.RED: "Red",
                           TrafficLight.YELLOW: "Red",
                           TrafficLight.GREEN : "Green"}
        return state_name_dict[state]

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        self.train_queue.enqueue(TrainItem(self.closest.state, self.behind, self.light_distance, cv_image))

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

        min_distance = float("inf")
        closest_index = 0
        if (self.waypoints):
            for wp_index in range(len(self.waypoints.waypoints)):
                waypoint_ps = self.waypoints.waypoints[wp_index].pose
                distance = euclidean_distance(pose.position.x,
                    pose.position.y,
                    waypoint_ps.pose.position.x,
                    waypoint_ps.pose.position.y)
                if (distance < min_distance):
                    distance = min_distance
                    closest_index = wp_index

        return closest_index


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

    def get_light_state(self):
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

        #Get classification
        pred, preds = self.light_classifier.get_classification(cv_image)
        return pred

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_index = None
        light_positions = self.config['light_positions']
        light_pose = Pose()
        closest_waypoint_to_light = None

        if(self.pose and self.waypoints):
            closest_waypoint_index = self.get_closest_waypoint(self.pose.pose)
            closest_waypoint_ps = self.waypoints.waypoints[closest_waypoint_index].pose

            #TODO find the closest visible traffic light (if one exists)
            closest_light_position = None
            closest_light_distance = float("inf")
            for light_position in self.config['light_positions']:
                distance = euclidean_distance(light_position[0], light_position[1], closest_waypoint_ps.pose.position.x, closest_waypoint_ps.pose.position.y)
                if distance < closest_light_distance:
                    closest_light_distance = distance
                    light_index = closest_waypoint_index
                    light_pose.position.x = light_position[0]
                    light_pose.position.y = light_position[1]
                    closest_waypoint_to_light = self.get_closest_waypoint(light_pose)


        state = self.get_light_state()
        print(closest_waypoint_to_light, state)
        return closest_waypoint_to_light, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
