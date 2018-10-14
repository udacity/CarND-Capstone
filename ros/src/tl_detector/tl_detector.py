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
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.lights = []
        self.has_image = None

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
        model_dir = None

        if self.config['is_site']:
            rospy.loginfo('Loading model for site.')
            model_dir = "./light_classification/models/site"
        else:
            rospy.loginfo('Loading model for sim mode.')
            model_dir = "./light_classification/models/simulation"

        model_file = "%s/inference_graph.pb" % model_dir
        self.light_classifier = TLClassifier(model_file)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0


        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            #light_wp, state = self.process_traffic_lights()
            #rospy.loginfo('TrafficLight %s',TrafficLight.RED)
            #rospy.loginfo('light_state: %s', light_wp)
            if self.pose and self.last_wp is not None:
                self.publish_light()
            rate.sleep()

    def publish_light(self):
        rospy.logerr("%s is getting published", self.last_wp)
        self.upcoming_red_light_pub.publish(self.last_wp)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):

        self.waypoints = waypoints
        if not self.waypoints_2d:
            #rospy.loginfo("inside waypoints_cb waypoints_2d")
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            #rospy.loginfo("waypoints_2d: %s, %s",self.waypoints_2d[0],self.waypoints_2d[1])
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.loginfo("lights are: %s", self.lights )

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        line_wp_idx, state = self.process_traffic_lights()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state

            if state == 0 :
                rospy.logerr('state red yes: %s', state)
                line_wp_idx = line_wp_idx
            else:
                rospy.logerr('state red no: %s', state)
                line_wp_idx = -1

            self.last_wp = line_wp_idx
        self.state_count += 1

    def get_closest_waypoint(self, pose_x, pose_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        x = pose_x
        y = pose_y

        closest_idx = None

        if self.waypoint_tree is not None:
            #rospy.loginfo("type waypoint_tree: %s", self.waypoint_tree.query([x,y],1)[1])
            closest_idx = self.waypoint_tree.query([x,y],1)[1]

            #Check if closest is ahead or behind vehicle
            closest_coord = self.waypoints_2d[closest_idx]
            prev_coord = self.waypoints_2d[closest_idx -1]

            #Equation for hyperplane through closest_coords
            cl_vect = np.array(closest_coord)
            prev_vect = np.array(prev_coord)
            pos_vect = np.array([x,y])

            val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

            if val > 0:
                closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.loginfo('light state: %s',light.state )

        # return light.state
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
        #light = None
        closest_light = None
        line_wp_idx = None
        #temp_wp_idx = None


        rospy.loginfo('process_traffic_lights used')

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        #TODO find the closest visible traffic light (if one exists)
        if self.waypoints is not None:
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                #get stop line waypoint idx
                line = stop_line_positions[i]
                traffic_light_wp_idx = self.get_closest_waypoint(line[0],line[1])
                #Find closest stop line waypoint index
                d = traffic_light_wp_idx - car_wp_idx
                if d  >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = traffic_light_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            rospy.loginfo('received state, %s', state.state)
            return line_wp_idx , state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
