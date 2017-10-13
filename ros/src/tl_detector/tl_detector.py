#!/usr/bin/env python
import rospy
import tf
import cv2
import yaml
import os
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from tl_helper import create_dir_if_nonexistent
from os.path import expanduser, join, exists
from kdtree import KDTree

STATE_COUNT_THRESHOLD = 3
VERBOSE = True

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

        rospy.Subscriber('/image_color', Image, self.collect_images_callback)

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

        # Parameters for collecting frames from the camera
        self.should_collect_data = True
        self.dump_images_dir = create_dir_if_nonexistent(join(expanduser('~'), 'traffic_light_dataset', 'raw_images'))
        self.dump_images_counter = len(os.listdir(self.dump_images_dir))
        self.last_dump_tstamp = rospy.get_time()

        # Used to find the closest waypoint
        self.kdtree = None

        rospy.spin()

    def collect_images_callback(self, msg):
        """
        Save camera images (currently once per second)
        """
        def should_collect_camera_image():
            return self.should_collect_data and (rospy.get_time() - self.last_dump_tstamp > 1)

        if should_collect_camera_image():

            # Convert image message to actual numpy data
            image_data = self.bridge.imgmsg_to_cv2(msg)
            image_data = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)  # opencv uses BGR convention
            image_path = join(self.dump_images_dir, '{:06d}.jpg'.format(self.dump_images_counter))

            # Dump image to dump directory
            cv2.imwrite(image_path, image_data)

            # Update counter and timestamp
            self.dump_images_counter += 1
            self.last_dump_tstamp = rospy.get_time()

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
        if (self.waypoints is not None and self.kdtree is None):
            if (VERBOSE):
                print ("initializing kdtree")
            points=[]
            i=0
            for waypoint in self.waypoints:
                points.append((float(waypoint.pose.pose.position.x),
                               float(waypoint.pose.pose.position.y),
                               int(i)))
                i += 1
            self.kdtree = KDTree(points)

        if (self.kdtree is not None):
            current_position = (pose.position.x, pose.position.y)
            closest = self.kdtree.closest_point(current_position)
            if (VERBOSE):
                print ("closest point to {} is {}".format(current_position, closest))
            return closest[2]

        return 0

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

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
