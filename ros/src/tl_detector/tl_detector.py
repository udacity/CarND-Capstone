#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifierCV

import tf
import cv2
import yaml
import os

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.create_train_data = rospy.get_param('generate_train',False)

        if self.create_train_data:
            self.train_data_dir = os.path.join(rospy.get_param('PATH'),'train')
            self.train_data_start_number = 1

            if self.create_train_data == True and not os.path.exists(self.train_data_dir):
                os.makedirs(self.train_data_dir)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        '''
        Input Subscribers
        '''
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)


        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        '''
        Vehicle/HW Configuration
        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)


        '''
        Output configuration
        '''
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        # Testing image queue for traffic light visualization
        self.upcoming_traffic_light_image_pub = rospy.Publisher('/traffic_light_image', Image, queue_size=1)


        self.bridge = CvBridge()
        self.light_classifier = TLClassifierCV()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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

            light_distance = 0
            if(self.last_wp != -1):
                light_distance = self.pose_distance(self.pose.pose,self.waypoints.waypoints[self.last_wp].pose.pose)
                rospy.loginfo("Publishing to Red Light - Distance at %0.2fm\n", light_distance)
            else:
                rospy.loginfo("Publishing to Red Light - No Lights")

            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            light_distance = 0
            if(self.last_wp != -1):
                light_distance = self.pose_distance(self.pose.pose,self.waypoints.waypoints[self.last_wp].pose.pose)
                rospy.loginfo("Publishing to Red Light - Distance at %0.2fm\n", light_distance)
            else:
                rospy.loginfo("Publishing to Red Light - No Lights")

            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_traffic_light(self, pose, light_positions):
        """Identifies the closest traffic light waypoint to the given waypoint.

        Args:
            pose (Pose): position to match a light to

        Returns:

            Light: the closest light from self.lights or None

        """


        if(light_positions):
            min_distance = 1e9
            min_light = None
            for ndx,light in enumerate(self.lights):
                distance = self.pose_distance(light.pose.pose,pose)
                if(distance < min_distance and self.is_waypoint_in_front_of_vehicle(light.pose,pose) ):
                    min_light = light
                    min_distance = distance


        #    rospy.loginfo("Closest Light At: %s\n m", min_distance)

            return min_light,min_distance
        #else
        return None

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if(self.waypoints and self.waypoints.waypoints ):
            min_distance = 1e9
            min_waypoint_ndx = 0
            for ndx,waypoint in enumerate(self.waypoints.waypoints):
                distance = self.pose_distance(waypoint.pose.pose,pose)
                if(distance < min_distance): #and self.is_waypoint_in_front_of_vehicle(waypoint.pose,pose)):
                    min_distance = distance
                    min_waypoint_ndx = ndx

            #rospy.loginfo("Closest Waypoint: %s \n At: %s\n m", self.waypoints.waypoints[min_waypoint_ndx].pose,min_distance)

            return min_waypoint_ndx
        #else
        return 0

    def is_waypoint_in_front_of_vehicle(self,waypoint,pose):
        """Returns if the waypoint is infront of the pose position. Pose position
        will need orientation values
        Args:
            waypoint (Pose): position to match a waypoint to
            pose (Pose): position to match a waypoint to

        Returns:
            boolean: if the waypoint is in the direction of travel from the pose

        """
        wp_x = waypoint.pose.position.x
        wp_y = waypoint.pose.position.y

        # vehicle orientation
        x_vec, y_vec,z_vec = self.get_vector_from_quaternion(pose.orientation)

        wp_dist = self.pose_distance(pose, waypoint.pose)

        vec_dist =  math.sqrt((wp_x-pose.position.x - x_vec*0.1)**2 + (wp_y-pose.position.y - y_vec*0.1)**2)

        return vec_dist < wp_dist

    def get_orientations_from_quaternion(self,q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return roll,pitch,yaw

    def get_vector_from_quaternion(self, q):
        roll, pitch, yaw = self.get_orientations_from_quaternion(q)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def pose_distance(self,pose_from, pose_to):
        #TODO Do we need 3D?
        """Calculates euclidean distance between two Pose objects (2d)
        Args:
            pose_from (Pose): position from calculation
            pose_to (Pose): position to calculation

        Returns:
            float: distance in m
        """
        return math.sqrt((pose_from.position.x-pose_to.position.x)**2 + (pose_from.position.y-pose_to.position.y)**2)

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


        camera_point=PointStamped()
        camera_point.header.frame_id = "/world"
        camera_point.header.stamp =rospy.Time(0)
        camera_point.point.x = point_in_world.x
        camera_point.point.y = point_in_world.y
        camera_point.point.z = point_in_world.z
        p = self.listener.transformPoint("/base_link",camera_point)

        #correct shift of traffic light to left / right because of car heading
        roll, pitch, yaw = self.get_orientations_from_quaternion(self.pose.pose.orientation)

        y_offset = p.point.x*math.sin(yaw)
        x = -(p.point.y + y_offset)/ p.point.x * fx + image_width*0.5

        y = 62 + image_height - (p.point.z / p.point.x * fy + image_height*0.5)

        return (int(x), int(y))


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
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        x, y = self.project_to_image_plane(light.pose.pose.position)

        if self.create_train_data:
            light_distance = self.pose_distance(self.pose.pose,light.pose.pose)
            if light_distance < 150:
                train_image_path = os.path.join(self.train_data_dir,'{0}.jpg'.format(self.train_data_start_number))
                cv2.imwrite(train_image_path, cv_image)
                self.train_data_start_number = self.train_data_start_number + 1

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
            light, light_distance = self.get_closest_traffic_light(self.pose.pose, light_positions)

        if light:
            light_wp_ndx = self.get_closest_waypoint(light.pose.pose)
            state = self.get_light_state(light)
            #rospy.loginfo("Closest Light at %0.2fm is Red? %s\n", light_distance, (state==TrafficLight.RED))
            return light_wp_ndx, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
