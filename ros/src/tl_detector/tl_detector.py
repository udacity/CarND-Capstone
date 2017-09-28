#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

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
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
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

        # initialize traffic light waypoints
        self.prev_nrst_wp = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        self.vehicle_orientation = msg.pose.orientation

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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
        if self.waypoints != None:
            self.wp_num = len(self.waypoints.waypoints)
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
            hd = lambda a, b: math.atan2((b.y-a.y), (b.x-a.x))
            smallest_dist = float('inf')
            

            for i in xrange(self.wp_num):
                
                wp_pos = self.waypoints.waypoints[i].pose.pose.position

                # distance between vehichle and the nearest waypoint
                dist = dl(pose.position, wp_pos)

                if dist < smallest_dist:
                    nearest_wp = i
                    smallest_dist = dist

            self.prev_nrst_wp = nearest_wp
            # if nearest_wp > 10696:
            #     self.prev_nrst_wp = 0
            return nearest_wp
        else:
            return 0

    def quaternion_to_rot(self, quaternion, trans):
        """ 
        according to:
        https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
        params:
        quaternion - angles in quaternions
        trans - transformation vector
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]
        t1 = trans[0]
        t2 = trans[1]
        t3 = trans[2]

        transform_mat = [   [1-(y*y+z*z), x*y-z*w, x*z+w*y, t1],
                            [x*y+w*z, 1-(x*x+z*z), y*z-w*x, t2],
                            [x*z-w*y, y*z+w*x, 1-(x*x+y*y), t3]]


        return transform_mat

    def get_classification(self, image):
        # Initial state
        state = TrafficLight.UNKNOWN

        # Match pixel area
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, np.array([150, 100, 150]), np.array([180, 255, 255]))
        extracted_image = cv2.bitwise_and(image, image, mask=mask_image)
        area = cv2.countNonZero(mask_image)

        # Check threshold
        pixels = 40

        if area > pixels:
            state = TrafficLight.RED

        # Return traffic light state - only UNKNOWN / RED

        return state

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image


        It seems there is a problem with this implmentation. Will consider...

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
        cam_m = [[fx, 0, image_width/2],[0, fy, image_height/2],[0, 0, 1]]
        trans_m = self.quaternion_to_rot(rot, trans)

        image_vec = np.matmul(cam_m, trans_m)
        worl_pos_vec = [point_in_world.x, point_in_world.y, point_in_world.z, 1]
        image_vec = np.matmul(image_vec, worl_pos_vec)



        x = int(image_vec[0])
        y = int(image_vec[1])

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

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)



        #TODO use light location to zoom in on traffic light in image

        #Get classification
        light_state = self.get_classification(cv_image)

        # rospy.logwarn("light state: %s", light_state)

        return self.light_classifier.get_classification(cv_image)

    def new_pose(self, x, y, z, yaw=0.):
        """Creates a new PoseStamped object - helper method for create_light
        Args:
            x (float): x coordinate of light
            y (float): y coordinate of light
            z (float): z coordinate of light
            yaw (float): angle of light around z axis
        Returns:
            pose (PoseStamped): new PoseStamped object
        """
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi*yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose


    def new_light(self, x, y, z, yaw, state):
        """Creates a new TrafficLight object
        Args:
            x (float): x coordinate of light
            y (float): y coordinate of light
            z (float): z coordinate of light
            yaw (float): angle of light around z axis
            state (int): ID of traffic light color (specified in styx_msgs/TrafficLight)
        Returns:
            light (TrafficLight): new TrafficLight object
        """
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = 'world'

        light.pose = self.new_pose(x, y, z, yaw)
        light.state = state

        return light


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = 1
        light_positions = self.config['light_positions']
        visible_distance = 90.0
        smallest_dist = float('inf')
        if self.waypoints != None:
            if(self.pose):
                try:
                    # waypoint of car
                    car_position = self.get_closest_waypoint(self.pose.pose)
                except Exception as e:
                    rospy.logwarn("Error: %s", e)
                    # pass
                else:
            
                    car_pose = self.waypoints.waypoints[car_position].pose.pose.position
                    #TODO find the closest visible traffic light (if one exists)
                    # nearest_light = 0

                    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
                    hdg = lambda a, b: math.atan2((b.y-a.y), (b.x-a.x))
                    
                    smallest_dist = float('inf')

                    for lights in light_positions:
                        light_tmp = self.new_light(lights[0], lights[1], 0., 0., TrafficLight.UNKNOWN)
                        light_pose = self.get_closest_waypoint(light_tmp.pose.pose)

                        dist = dl(car_pose, light_tmp.pose.pose.position)

                        if dist < smallest_dist:
                            smallest_dist = dist
                            light = light_tmp
                            light_wp = light_pose


            if light and smallest_dist < visible_distance and car_position < light_wp:
                try:
                    state = self.get_light_state(light)
                    return light_wp, state
                except Exception as e:
                    rospy.logwarn("Error: %s", e)
            # self.waypoints = None
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
