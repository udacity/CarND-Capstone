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
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3
IMAGE_COUNT_THRESHOLD = 10

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
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

        # Publisher for red light waypoint position
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        # Define traffic light classifier with location of detection model
        self.light_classifier = TLClassifier('./light_classification/tl_detection')
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.img_count = IMAGE_COUNT_THRESHOLD
        self.last_detected_state = TrafficLight.UNKNOWN

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_cb(self, msg):
        """Gets current pose of the vehicle"""
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """Stores base waypoints as a KDTree"""
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        """Gets traffic light positions"""
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

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light, diff):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify
            diff: distance in waypoints to next traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        # Pass through the model 1 image every IMAGE_COUNT_THRESHOLD images
        if self.img_count >= IMAGE_COUNT_THRESHOLD:

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            # Crop relevant portion of image depending on distance to traffic light
            height = cv_image.shape[0]
            if diff > 100:
                cv_image = cv_image[int(height*.75):height]
            elif diff > 40:
                cv_image = cv_image[int(height*.5):height]

            # Check if detects traffic lights
            detected = self.light_classifier.get_classification(cv_image)

            # Reset image counter
            self.img_count = 0

            # If traffic light detected return state
            if detected:
                #rospy.loginfo("Traffic light detected. State: ")
                #rospy.loginfo(light.state)
                self.last_detected_state = light.state
                return light.state
            else:
                return TrafficLight.UNKNOWN

        # Return last detected state if not reached IMAGE_COUNT_THRESHOLD
        else:
            self.img_count += 1
            return self.last_detected_state

        # Get classification
        #return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        # Find the closest visible traffic light (if one exists)
        diff = len(self.waypoints.waypoints)

        # Loop through traffic light positions
        for i, light in enumerate(self.lights):
            # Get stop line waypoint index
            line = stop_line_positions[i]
            temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

            # Find closest line
            d = temp_wp_idx - car_position
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                line_wp_idx = temp_wp_idx

        # if found a closest light and is 300 waypoints in front of us
        if closest_light and diff < 300:
            # get state of traffic light
            state = self.get_light_state(closest_light, diff)
            # return line waypoint index & traffic light state
            return line_wp_idx, state

        # else, no upcoming traffic light was found
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
