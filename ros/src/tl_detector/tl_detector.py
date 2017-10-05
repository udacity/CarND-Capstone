#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import time
import math

import numpy as np

STATE_COUNT_THRESHOLD = 0 #3 #NOTE: change back to 3 after training/testing
DEBUG = True


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
        self.unity_sim = rospy.get_param("unity_sim", False)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.poses = []
        self.lights_list = []

        rospy.spin()

    def pose_cb(self, msg):
        self.poses.append(msg)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights_list.append(msg)

    def find_closest(self, time, input_list):
        while len(input_list) > 1:
            item = input_list.pop(0)
            if (item.header.stamp > time):
                return item
        if input_list:
            return input_list.pop()
        return None

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #sync other inputs
        self.pose = self.find_closest(msg.header.stamp,self.poses)
        sync_lights_msg = self.find_closest(msg.header.stamp,self.lights_list)
        if sync_lights_msg:
            self.lights = sync_lights_msg.lights

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        print "state: ", state, " red: ", TrafficLight.RED
        if self.state != state:
            self.state_count = 0
            self.state = state
        if self.state_count >= STATE_COUNT_THRESHOLD:
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
        # TODO implement
        # DONE by Facheng Li.

        min_distance = None
        min_dist_ind = -1
        if self.waypoints is not None:
            for i, wp in enumerate(self.waypoints.waypoints):
                distance = (wp.pose.pose.position.x - pose.position.x) ** 2 + \
                           (wp.pose.pose.position.y - pose.position.y) ** 2
                if distance < min_distance or min_distance is None:
                    min_distance = distance
                    min_dist_ind = i
        else:
            print "No waypoints in TL Detector"
        return min_dist_ind

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
        '''
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

        # TODO Use tranform and rotation to calculate 2D position of light in image
        # DONE by Facheng Li

        RT = np.mat(self.listener.fromTranslationRotation(trans, rot))

        camMat = np.mat([[fx, 0, image_width / 2],
                         [0, fy, image_height / 2],
                         [0, 0, 1]])

        point_3d = np.mat([[point_in_world.x], [point_in_world.y],
                              [point_in_world.z], [1.0]])
        point_2d = camMat * (RT * point_3d)[:-1, :]

        x = int(point_2d[0, 0] / point_2d[2, 0])
        y = int(point_2d[1, 0] / point_2d[2, 0])
        '''
        #used equations and parameters from https://discussions.udacity.com/t/focal-length-wrong/358568/23
        x_offset = 0
        y_offset = 0
        if self.unity_sim:
            x_offset = (image_width / 2) - 30
            y_offset = image_height + 50
        ##########################################################################################

        # get transform between pose of camera and world frame
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                                           "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                                                           "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return (0, 0, 0, 0)

        # car yaw angle
        yaw = tf.transformations.euler_from_quaternion(rot)[2]

        # Rotate the coordinate space on the z axis to align relative to car
        world_x = point_in_world.x
        world_y = point_in_world.y
        world_z = point_in_world.z

        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)

        yaw_oriented_point = (world_x * cos_yaw - world_y * sin_yaw,
                              world_x * sin_yaw + world_y * cos_yaw,
                              world_z)

        # Apply transformation to center on car position
        car_rel_x = yaw_oriented_point[0] + trans[0]
        car_rel_y = yaw_oriented_point[1] + trans[1]
        car_rel_z = yaw_oriented_point[2] + trans[2]

        # rotate to camera view space with offset
        camera_height_offset = 1.1
        camera_rel_x = -car_rel_y
        camera_rel_z = car_rel_x
        camera_rel_y = -(car_rel_z - camera_height_offset)

        # apply focal length and offsets
        #  center
        center_x = int((camera_rel_x * fx / camera_rel_z) + x_offset)
        center_y = int((camera_rel_y * fy / camera_rel_z) + y_offset)

        corner_offset = 1.5

        #  top left
        top_x = int(((camera_rel_x - corner_offset) * fx / camera_rel_z) + x_offset)
        top_y = int(((camera_rel_y - corner_offset) * fy / camera_rel_z) + y_offset)

        #  bottom right
        bottom_x = int(((camera_rel_x + corner_offset) * fx / camera_rel_z) + x_offset)
        bottom_y = int(((camera_rel_y + corner_offset) * fy / camera_rel_z) + y_offset)

        return (top_x, top_y, bottom_x, bottom_y)


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x0, y0, x1, y1 = self.project_to_image_plane(light.pose.pose.position)
        
        h, w, _ = cv_image.shape
        
        if (x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0 or
            x0 > w or x1 > w or y0 > h or y1 > h):
            return TrafficLight.UNKNOWN

        # TODO use light location to zoom in on traffic light in image

        
        
        # TODO modify it to extract light image according to light distance
        #x0, x1 = max(x-16, 0), min(x+16, w)
        #y0, y1 = max(y-16, 0), min(y+16, h)
        #x0, x1 = max(x-16, 0), min(x+16, w)
        #y0, y1 = max(y-0, 0), min(y+96, h)

        im_light = cv_image[y0:y1, x0:x1, :]

        #write image to file
        base_dir = '/home/marv/data/tl/'
        current_time_str = str(int(time.time()*1000))
        if (light.state == TrafficLight.RED):
            img_save_path = base_dir + 'red/udacity_img' + current_time_str + '.jpg'
        elif (light.state == TrafficLight.GREEN):
            img_save_path = base_dir + 'green/udacity_img' + current_time_str + '.jpg'
        elif (light.state == TrafficLight.YELLOW):
            img_save_path = base_dir + 'yellow/udacity_img' + current_time_str + '.jpg'
        else:
            img_save_path = base_dir + 'unknown/udacity_img' + current_time_str + '.jpg'

        if DEBUG:
            cv2.imwrite(img_save_path,im_light)

        # Get classification
        tl_class = self.light_classifier.get_classification(im_light)
       
        return tl_class

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = None
        stop_pose = Pose()
        visible = False

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if (self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if DEBUG:
                print "Car position is: ", car_position
            # TODO find the closest visible traffic light (if one exists)


            for light_i in self.lights:

                light_pos_i = np.array([light_i.pose.pose.position.x,
                                        light_i.pose.pose.position.y])
                min_dist = 1000000
                for stop_pos_j in stop_line_positions:
                    stop_pos_j = np.array(stop_pos_j)
                    dist = np.sum((stop_pos_j - light_pos_i) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        stop_pose.position.x = stop_pos_j[0]
                        stop_pose.position.y = stop_pos_j[1]

                light_wp_i = self.get_closest_waypoint(stop_pose)
                if light_wp_i >= car_position and light_wp_i != -1:
                    if light_wp is None or light_wp_i < light_wp:
                        light_wp = light_wp_i
                        light = light_i

            # if light_wp is close car_position, then light will be visible
            if light_wp is not None and light_wp - car_position < 100:
                visible = True

        if light is not None and visible:
            #determine image latency
            image_time = self.camera_image.header.stamp
            light_time = light.header.stamp
            #print ("image latency is: ", (light_time-image_time).to_sec())
            
            #get state
            state = self.get_light_state(light)
            #print("Inferred Traffic Light state is: ", state)
            #KR testing
            state = light.state
            #print("Correct TL State is: ", state)
            return light_wp, state
        #Why are we setting self.waypoints to None?
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
