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
        self.has_image = False

        self.collect_training_data = True
        self.training_data_counter = 0
        self.state_file = open("../../../training_data/state.txt","w")

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        # Plot current pose
        monitor = False
        if monitor:
            rospy.logerr("Seq: {}: X: {}, Y: {}, Orientation: {}".format(
                self.pose.header.seq,
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.pose.pose.orientation.w))

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        # rospy.logerr("X: {}".format(self.waypoints.waypoints[0].pose.pose.position.x))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def save_data(self):
        if self.pose == None:
            return
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv2.imwrite("../../../training_data/data{:06d}.png".format(self.training_data_counter), cv_image)
        self.state_file.write("{}".format(0))
        #self.state_file.write("Current POSE: {}\nEND POSE\n".format(self.pose))
        #self.state_file.write("LIGHTS: {}\nEND LIGHTS\n".format(self.lights))
        rospy.logerr("Curr POSE:")
        rospy.logerr(self.pose)
        rospy.logerr("\n")
        rospy.logerr("LIGHTS:")
        rospy.logerr(self.lights)

        self.training_data_counter += 1
        self.collect_training_data = False

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        self.camera_image = msg
        self.has_image = True

        if self.collect_training_data:
            self.save_data()

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

    def get_next(self, base_pose, pose_list):
        """
        Returns index of the next list entry to base_pose
        :param base_pose: Single Pose (e.g. current pose)
        :param pose_list: List with poses to search for the closest
        :return: Index of closest list entry
        """
        closest_dist = float("inf")
        closest_index = 0

        for i in range(0, len(pose_list)):
            # Check if pose in list in in front of the vehicle
            if self.check_is_ahead(base_pose, pose_list[i]):

                # Calculate the distance between pose und pose in list
                dist = self.squared_dist(base_pose, pose_list[i])
                # If distance is smaller than last saved distance
                if dist < closest_dist:
                    # Save
                    closest_dist = dist
                    closest_index = i

    def check_is_ahead(self, pose_1, pose_2):
        # TODO: Use vehicle orientation to identify which points are ahead
        return True

    def squared_dist(self, pose_1, pose_2):
        dx = pose_1.position.x - pose_2.position.x
        dy = pose_1.position.y - pose_2.position.y
        return dx*dx + dy*dy

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
