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
from scipy.spatial import KDTree
import time
import thread


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []
        self.has_image = False
        self.thread_working = False

        self.frame_count = 0


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, )

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.detection_traffic_light_pub = rospy.Publisher('/tl_detection', Image, queue_size=1)



        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights


    def detect_tl(self):
        #rospy.loginfo("Detection start")
        start = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        box_cv_image = self.light_classifier.detect_traffic_lights(cv_image)
        
        end = time.time()

        rospy.loginfo("Detection Time:%f s", end - start)

        ''''''
        try:
            image_message = self.bridge.cv2_to_imgmsg(box_cv_image, encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        self.detection_traffic_light_pub.publish(image_message)

        self.thread_working = False

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo("Image_cb")
        if not self.thread_working:
            self.thread_working = True
            
            self.camera_image = msg
            
            
            thread.start_new_thread( self.detect_tl, ())
            #box_image = self.light_classifier.detect_traffic_lights(cv_image)

            #pic_filename = "./result/%08d.png"%self.frame_count
            #cv2.imwrite(pic_filename, box_image)
            #self.frame_count += 1

            #image_message = self.bridge.cv2_to_imgmsg(box_image, encoding="passthrough")
            
            
            #self.upcoming_red_light_pub.publish(Image(image_message))

            #light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            
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
            '''
            

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #For test in simulator
        #return light.state

        ''''''

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")



        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        return light.state
        

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.base_waypoints.waypoints)
            for i in range(len(self.lights)):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = self.lights[i]
                    line_wp_idx = temp_wp_idx



        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
            
        self.base_waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
