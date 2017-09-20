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
import waypoint_lib.helper as helper
import os.path
import message_filters

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None

        self.waypoints = None
        self.camera_image = None
        self.camera_image_prev_seq = None
        self.lights = []


        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        image_sub = message_filters.Subscriber('/image_color', Image)
        pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        lights_sub = message_filters.Subscriber('/vehicle/traffic_lights', TrafficLightArray)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, pose_sub, lights_sub], 10, 0.005)
        # ts.registerCallback(self.image_sync)

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

        self.record_cnt = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        # rospy.loginfo('>>> got pose')

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # rospy.loginfo('>>> got waypoints')

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # rospy.loginfo('>>> got traffic_lights')

    def image_sync(self, image_msg, pose_msg, lights_msg):
        rospy.loginfo('---- IMAGE SYNC ------')

        lights_msg = lights_msg.lights

        # Select the closest waypoint from lights array which was received from /vehicle/traffic_lights topic
        if (self.waypoints and self.record_cnt % 1 == 0 and image_msg.header.seq != self.camera_image_prev_seq):

            self.camera_image_prev_seq = image_msg.header.seq

            car_wp = helper.next_waypoint_idx(pose_msg, self.waypoints.waypoints)

            lights_wp = [helper.closest_waypoint_idx(l.pose, self.waypoints.waypoints) for l in lights_msg]
            lights_dists = [helper.wp_distance(car_wp, lwp, self.waypoints.waypoints) for lwp in lights_wp]
            closest_light = lights_dists.index(min(lights_dists))

            rospy.loginfo('--- car_wp = {}'.format(car_wp))
            rospy.loginfo('--- closest_light[{}] = {}, {}'.format(closest_light, lights_msg[closest_light].state, lights_dists[closest_light]))

            # light = lights_wp[closest_light]
            light_wp = lights_wp[closest_light]
            light = lights_msg[closest_light]

            # This we have only in simulator for testing
            state = lights_msg[closest_light].state
            rospy.loginfo('--- SIM: closest_light_wp = {}, state = {}'.format(light_wp, light.state))

            # Collect test data
            # self.record_camera_image(light, state)
            rospy.loginfo('--- =============== saving image ....')

            # Check folder exists
            img_folder = os.path.join('.', 'output_images')
            if not os.path.exists(img_folder):
                os.makedirs(img_folder)
            # rospy.loginfo("img_folder = {}".format(os.path.dirname(os.path.abspath(img_folder))))

            car_wp = helper.next_waypoint_idx(pose_msg, self.waypoints.waypoints)
            light_wp = helper.closest_waypoint_idx(light.pose, self.waypoints.waypoints)

            # Is light wisible?
            waypoints_num = len(self.waypoints.waypoints)
            light_dist = (light_wp - car_wp + waypoints_num) % waypoints_num

            # Save image
            if 40 < light_dist < 160:
                img_filename = os.path.join(img_folder, '{:06d}-{}-{}.png'.format(self.record_cnt, state, light_dist))
                rospy.loginfo("--- saving: {}, dist = {}".format(img_filename, light_dist))
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                cv2.imwrite(img_filename, cv_image)
            else:
                rospy.loginfo("--- light is far away: {}".format(light_dist))

        self.record_cnt += 1

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        # rospy.loginfo('>>> got image')



        light_wp, state = self.process_traffic_lights()

        '''
        rospy.loginfo('some my processing')

        # Select the closest waypoint from lights array.
        if (self.lights and self.waypoints):

            car_wp = helper.next_waypoint_idx(self.pose, self.waypoints.waypoints)

            lights_wp = [helper.closest_waypoint_idx(l.pose, self.waypoints.waypoints) for l in self.lights]
            lights_dists = [helper.wp_distance(car_wp, lwp, self.waypoints.waypoints) for lwp in lights_wp]
            closest_light = lights_dists.index(min(lights_dists))

            rospy.loginfo('car_wp = {}'.format(car_wp))
            rospy.loginfo('closest_light[{}] = {}, {}'.format(closest_light, self.lights[closest_light].state, lights_dists[closest_light]))
            rospy.loginfo('closest_light_wp = {}, state = {}'.format(lights_wp[closest_light], self.lights[closest_light].state))

            light_wp = lights_wp[closest_light]
            state = self.lights[closest_light].state
        '''

            # for i, l in enumerate(self.lights):
            #     # l = self.lights[i]
            #     rospy.loginfo('[{}] = {}: {}, {}'.format(i, l.state, l.pose.pose.position.x, l.pose.pose.position.y))
            #     if l.state > TrafficLight.RED:
            #         rospy.loginfo("NOT REEEEEEEEEEEEEEEEEED")

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

    def record_camera_image(self, light, state):
        pass
        '''
        if(not self.has_image):
            return

        if self.record_cnt % 10 == 0:

            rospy.loginfo('=============== saving image ....')

            # Check folder exists
            img_folder = os.path.join('.', 'output_images')
            if not os.path.exists(img_folder):
                os.makedirs(img_folder)
            # rospy.loginfo("img_folder = {}".format(os.path.dirname(os.path.abspath(img_folder))))

            car_wp = helper.next_waypoint_idx(self.pose, self.waypoints.waypoints)
            light_wp = helper.closest_waypoint_idx(light.pose, self.waypoints.waypoints)

            # Is light wisible?
            waypoints_num = len(self.waypoints.waypoints)
            light_dist = (light_wp - car_wp + waypoints_num) % waypoints_num

            # Save image
            if 20 < light_dist < 200:
                img_filename = os.path.join(img_folder, '{:06d}-{}-{}.png'.format(self.record_cnt, state, light_dist))
                rospy.loginfo("saving: {}, dist = {}".format(img_filename, light_dist))
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                cv2.imwrite(img_filename, cv_image)
            else:
                rospy.loginfo("light is far away: {}".format(light_dist))


        self.record_cnt += 1
        '''


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
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1
        light_positions = self.config['light_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)\

        '''
        # Select the closest waypoint from lights array which was received from /vehicle/traffic_lights topic
        if (self.lights and self.waypoints):

            car_wp = helper.next_waypoint_idx(self.pose, self.waypoints.waypoints)

            lights_wp = [helper.closest_waypoint_idx(l.pose, self.waypoints.waypoints) for l in self.lights]
            lights_dists = [helper.wp_distance(car_wp, lwp, self.waypoints.waypoints) for lwp in lights_wp]
            closest_light = lights_dists.index(min(lights_dists))

            rospy.loginfo('car_wp = {}'.format(car_wp))
            rospy.loginfo('closest_light[{}] = {}, {}'.format(closest_light, self.lights[closest_light].state, lights_dists[closest_light]))

            # light = lights_wp[closest_light]
            light_wp = lights_wp[closest_light]
            light = self.lights[closest_light]

            # This we have only in simulator for testing
            state = self.lights[closest_light].state
            rospy.loginfo('SIM: closest_light_wp = {}, state = {}'.format(light_wp, light.state))

            # Collect test data
            # self.record_camera_image(light, state)
        '''


        if light:
            # state = self.get_light_state(light)
            return light_wp, state
        # self.waypoints = None # don't know why this line is here [Pavlo]
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
