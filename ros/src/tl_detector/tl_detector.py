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
        self.waypoints = []
        self.camera_image = None
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

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

    def get_closest_waypoint(self, pos_x, pos_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Done
        if self.waypoints is None:
            return None
        min_distance = 9999999.
        closed_indx = 0

        for indx, waypoint in enumerate(self.waypoints):
            waypoint_pos_x = waypoint.pose.pose.position.x
            waypoint_pos_y = waypoint.pose.pose.position.y

            distance = math.sqrt((waypoint_pos_x - pos_x)**2 + (waypoint_pos_y - pos_y)**2)

            if distance < min_distance:
                min_distance = distance
                closed_indx = indx
            min_distance = min(min_distance, distance)

        # rospy.loginfo('The close waypoint is: [' + str(self.waypoints[closed_indx].pose.pose.position.x) + ', '
        #               + str(self.waypoints[closed_indx].pose.pose.position.y) + ']')
        # rospy.loginfo('Car position is: [' + str(pos_x) + ', ' + str(pos_y))
        # rospy.loginfo('Distance is: ' + str(min_distance))
        # rospy.loginfo('============================')
        return closed_indx

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
        ahead_stop_line_wp = -1
        # List of positions that correspond to the line to stop in front of for a given intersection

        if(self.pose):
            car_waypoint = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            stop_line_waypoints = self.get_stopline_waypoints()

            # find index of waypoint for the stop line ahead
            for stop_line_wp in stop_line_waypoints:
                if car_waypoint < stop_line_wp:
                    ahead_stop_line_wp = stop_line_wp
                    break
            # rospy.loginfo(str(ahead_stop_line_wp) +' '+ str(self.waypoints[ahead_stop_line_wp].pose.pose.position.x)
            #               + ' ,' + str(self.waypoints[ahead_stop_line_wp].pose.pose.position.y))
        # TODO: light state detection need to be complete
        if light:
            state = self.get_light_state(light)

        return ahead_stop_line_wp, TrafficLight.UNKNOWN


    def get_stopline_waypoints(self):
        stop_line_wp = []
        for stop_line in self.stop_line_positions:
            x = stop_line[0]
            y = stop_line[1]
            indx = self.get_closest_waypoint(x, y)
            stop_line_wp.append(indx)
        return stop_line_wp

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
