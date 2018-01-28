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

STATE_COUNT_THRESHOLD = 4

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.tl_poses = list()       # traffic light waypoint positions in Pose object
        self.tl_wp_indices = list()  # traffic light waypoint indices initialization to empty list

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
        self.light_classifier = TLClassifier(simulator=True)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.wp2light = -1

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.set_tl_wp_indices()  # precomputes waypoints of tarffic lights


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



    def set_tl_wp_indices(self):
        # AP 1/21/18
        # tl = traffic light
        # pre-maps each traffic light to a base waypoint

        if self.waypoints is None:
            return

        # traffic light positions
        tl_positions = self.config['stop_line_positions']

        self.tl_poses = list()       # traffic light waypoint positions in Pose object
        self.tl_wp_indices = list()  # traffic light waypoint indices initialization to empty list

        #rospy.loginfo('length of pos = ',type(tl_positions))


        for tlp in tl_positions:
            tl_pose = Pose()
            tl_pose.position.x = tlp[0]   #convert tl_position to Pose
            tl_pose.position.y = tlp[1]   #convert tl_position to Pose
            self.tl_poses.append(tl_pose)
            #ll = len(self.tl_poses)
            #rospy.loginfo("Pose: %f %f", tl_pose.position.x, tl_pose.position.y)
            #for i in range(ll):
            #    rospy.loginfo("list: %f %f", self.tl_poses[i].position.x, self.tl_poses[i].position.y)
            #rospy.loginfo("------")
            #rospy.loginfo(self.tl_poses)
            temp = self.get_closest_waypoint(tl_pose)
            self.tl_wp_indices.append(temp)

        #rospy.loginfo('tl poses: %d',len(self.tl_poses))
        #for i in self.tl_wp_indices:
        #    rospy.loginfo('set: i= %d', i)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if self.waypoints is None:
            #rospy.loginfo("waypoints is None")
            return -1

        min_dist  = 10000
        min_index = -1

        pos_x = pose.position.x
        pos_y = pose.position.y
        # check all the waypoints to see which one is the closest to our current position
        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = math.sqrt((pos_x - wp_x) ** 2 + (pos_y - wp_y) ** 2)
            if (dist < min_dist):  # we found a closer wp
                min_index = i  # we store the index of the closest waypoint
                min_dist = dist  # we save the distance of the closest waypoint

        # returns the index of the closest waypoint
        return min_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        #if light too far away do not bother
        if self.wp2light > 200:
            return 4

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

        # if  car position is available then get nearest waypoint
        if (self.pose):
            car_wp = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN


        #TODO find the closest visible traffic light (if one exists)
        min_wp_distance = 100000  # large distance --- waypoint units
        light = None
        l_wp = -1  # so far nothing detected
        for i, tl_wp in enumerate(self.tl_wp_indices):  # for each waypoint index of traffic light
            wp_distance = tl_wp - car_wp  # distance between car and traffic light, wp=waypoint
            cond1 = wp_distance > 0  # light is in front
            cond2 = (wp_distance < min_wp_distance)  # new minimum found
            if (cond1 and cond2):  # choose wp_distance
                min_wp_distance = wp_distance
                l_wp = tl_wp
                light = self.tl_poses[i]

        if (l_wp == -1):
            self.wp2light = 10000   #large number to ignore image
        else:
            self.wp2light = l_wp - car_wp


        if light:
            state = self.get_light_state(light)
            rospy.loginfo('process_tl: car-wp=%d, light-wp=%d, d=%d state=%d', car_wp, l_wp, self.wp2light, state)
            return l_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
