#!/usr/bin/env python
import os
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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.data_collection = False
        self.ground_truth = False
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []
        self.waypoints_2d = None
        self.waypoints_tree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=2)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(is_site=self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        if self.data_collection:
            self.images_path = '/home/udacity-ros/data/udacity-simulator-data/simulator-images/'
            self.img_counter = {}
            folders = ['0', '1', '2', '4']
            for folder in folders:
                folder_path = os.path.join(self.images_path, folder)
                try:
                    last_image_name = sorted(os.listdir(folder_path), reverse=True)[0]
                    last_idx = int(last_image_name.split('.')[0][3:]) # get number XXXXX from 'imgXXXXX.jpg' file name
                    self.img_counter[folder] = last_idx + 1
                    print(folder+' image counter - %d' % self.img_counter[folder])
                except:
                    if not os.path.exists(folder_path):
                        os.mkdir(folder_path)
                    self.img_counter[folder] = 0
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if self.waypoints_2d is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        self.lights = msg.lights
        #D = {0: "RED", 1: "YELLOW", 2: "GREEN", 4: "UNKNOWN"}
        if self.ground_truth and not self.has_image:
            light_wp, state = self.process_traffic_lights()
            # rospy.loginfo("state=%d, red=%d", state, TrafficLight.RED)
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.upcoming_red_light_pub.publish(Int32(light_wp))


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        if self.data_collection and self.ground_truth:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            label = state
            if light_wp - car_wp_idx > 200:
                label = 4

            label = str(label)
            img_name = label+'/img%05d.jpg' % self.img_counter[label]
            save_path = self.images_path + img_name
            cv2.imwrite(save_path, cv_image)
            self.img_counter[label] += 1
            rospy.loginfo("[tl_detector]current wp = %d, next red light = %d w state %d. saved as %d" % (car_wp_idx, light_wp, state, self.img_counter[label])) 
            
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

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.ground_truth:
            return light.state

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
        closest_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            if self.waypoints is not None:
                diff = len(self.waypoints.waypoints)
                for i, light in enumerate(self.lights):
                    # rospy.loginfo("%d : state %d", i, light.state)
                    x, y = stop_line_positions[i]
                    temp_wp_idx = self.get_closest_waypoint(x, y)
                    d = temp_wp_idx - car_wp_idx
                    if 0 <= d < diff and d < 150:
                        diff = d
                        closest_light = light
                        line_wp_idx = temp_wp_idx

        if closest_light is not None:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        else:
            rospy.loginfo("[tl_detector], No traffic light expected nearby")

        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
