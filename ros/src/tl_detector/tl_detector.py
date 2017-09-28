#!/usr/bin/env python
import rospy
from datetime import datetime, timedelta
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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

        self.light_dict = None
        self.light_waypoints = None

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()

        self.state = TrafficLight.UNKNOWN
        self.state_count = 0
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1

        self.image_date = datetime.now()

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


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # --- Find all waypoints for traffic light.
        if (not self.light_waypoints or not self.light_dict) and self.waypoints:
            self.light_dict = {}
            for idx, alight in enumerate(self.lights):
                alight_wp = self.get_closest_waypoint(alight.pose.pose)
                self.light_dict[alight_wp] = alight
            self.light_waypoints = sorted(self.light_dict.keys())

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # Skip certain images to lower the processing rate.
        # TODO: Tune skip time.
        now = datetime.now()
        if now - self.image_date < timedelta(milliseconds=500):
            # Assume same as previous.
            light_wp, state = self.last_wp, self.state
        else:
            self.image_date = now
            # Process frame.
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
        closest_gap = float('inf')
        closest_idx = 0
        for idx, awp in enumerate(self.waypoints):
            agap = (awp.pose.pose.position.x - pose.position.x)**2 + \
                   (awp.pose.pose.position.y - pose.position.y)**2
            if agap < closest_gap:
                closest_gap = agap
                closest_idx = idx
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        prediction = self.light_classifier.get_classification(cv_image)
        # print '--->', prediction
        return prediction

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_positions = self.config['stop_line_positions']

        if(self.pose and self.waypoints and self.lights and self.light_waypoints and self.light_dict):
            car_wp = self.get_closest_waypoint(self.pose)

            # --- Determine the next traffice light in waypoint.
            light = None
            light_wp = None

            if car_wp <= self.light_waypoints[0]:
                light_wp = self.light_waypoints[0] # Before the first light waypoint.
            elif car_wp >= self.light_waypoints[-1]:
                light_wp = self.light_waypoints[0] # After the last light waypoint. So loop around.
            else:
                for light1_wp, light2_wp in zip(self.light_waypoints, self.light_waypoints[1:]):
                    if car_wp > light1_wp and car_wp <= light2_wp:
                        light_wp = light2_wp

            # --- Get next traffic light ahead.
            light = self.light_dict.get(light_wp)

            # Found light_wp and if it is close enough.
            if light and (light_wp - car_wp) < 60:
                state = self.get_light_state(light)
                # --- TESTING ONLY
                # state = light.state
                # --- TESTING ONLY
                return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
