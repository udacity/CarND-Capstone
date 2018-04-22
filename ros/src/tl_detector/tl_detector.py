#!/usr/bin/env python
import numpy as np
import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight

STATE_COUNT_THRESHOLD = 5
STOP_LINE_DISTANCE = 100


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.velocity = 0
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = None
        self.lights_2d = None
        self.stop_line_positions = None
        self.stop_line_positions_tree = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''


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

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def velocity_cb(self, msg):
        self.velocity = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        self.lights_2d = [[light.pose.pose.position.x, light.pose.pose.position.y] for light in msg.lights]
        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_positions_tree = KDTree(self.stop_line_positions)

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
            pose (Pose): position to match a stop_line to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if not self.waypoints_tree:
            return -1

        x = pose[0]
        y = pose[1]
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - pos_vect, pos_vect - prev_vect)

        if val > 0:
            closest_idx = (closest_idx - 1) % len(self.waypoints_2d)
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        return light.state
        #if (not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Generate sample images
        # import time
        # import cv2
        # filename = '/capstone/data/light_samples/%d/%f.jpg' % (light.state, time.time())
        # cv2.imwrite(filename, cv_image)
        # rospy.logerr('Sample image saved: %s', filename)

        # Get classification
        #return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_idx = -1

        if self.pose and self.stop_line_positions_tree:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            min_distance_to_stop_line = STOP_LINE_DISTANCE + 1

            # Find all the stop lines in 100m
            closest_idx_list = self.stop_line_positions_tree.query_ball_point([x, y], STOP_LINE_DISTANCE)

            for closest_idx in closest_idx_list:
                light_vect = np.array(self.lights_2d[closest_idx])
                stop_line_vect = np.array(self.stop_line_positions[closest_idx])
                pos_vect = np.array([x, y])

                val = np.dot(stop_line_vect - pos_vect, light_vect - stop_line_vect)

                if val > 0:
                    distance = np.linalg.norm(stop_line_vect - pos_vect)
                    if distance < min_distance_to_stop_line:
                        min_distance_to_stop_line = distance
                        light_idx = closest_idx

        # TODO find the closest visible traffic light (if one exists)
        if light_idx >= 0 and self.waypoints_2d:
            light = self.lights[light_idx]
            light_wp = self.get_closest_waypoint(self.stop_line_positions[light_idx])

            state = self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
