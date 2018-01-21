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
import sys
import math
import time
import numpy as np

STATE_COUNT_THRESHOLD = 3
MEASURE_PERFORMANCE = False
DISPLAY_CAMERA = False

proc_time_sum = 0
proc_iterations = 0

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size = 1)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = 1)
        
        rospy.spin()
    
    
    def pose_cb(self, msg):
        #self.pose = msg
        self.pose = msg.pose
    
    
    def waypoints_cb(self, waypoints):
        #self.waypoints = waypoints
        self.waypoints = waypoints.waypoints
        self.waypoints_array = np.asarray([(w.pose.pose.position.x, w.pose.pose.position.y) for w in waypoints.waypoints])
        if rospy.get_param('/unregister_base_waypoints', False):
            self.sub2.unregister()
            rospy.loginfo('base_waypoints subscriber unregistered')
    
    
    def traffic_cb(self, msg):
        self.lights = msg.lights
    
    
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        
        Args:
            msg (Image): image from car-mounted camera
        """
        global proc_time_sum, proc_iterations
        
        if MEASURE_PERFORMANCE:
            start_time = time.time()
        
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        
        '''
        Publish upcoming red lights at camera frequency.
        #Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        Each predicted state has to occur `~state_count_threshold` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        
        #elif self.state_count >= STATE_COUNT_THRESHOLD:
        elif self.state_count >= rospy.get_param('~state_count_threshold', 3):
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        
        self.state_count += 1
        
        if MEASURE_PERFORMANCE:
            end_time = time.time()
            duration = end_time - start_time
            proc_time_sum += duration
            proc_iterations += 1
            rospy.loginfo("Processing time of image_cb(): " + str(duration) + " average: " + str(proc_time_sum / proc_iterations))
    
    
    #def get_closest_waypoint(self, pose):
    def get_closest_waypoint(self, pos_x, pos_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        # TODO implement
        index = -1
        
        # Return if way points are empty
        if self.waypoints is None:
            return index
        
        pos = np.asarray([pos_x, pos_y])
        dist_squared = np.sum((self.waypoints_array - pos)**2, axis = 1)
        index = np.argmin(dist_squared)
        #rospy.loginfo('tl.detector.get_closest_waypoint({}) found at = {}, distance = {}, time = {}'.format(
        #    position, index, np.sqrt(dist_squared[index]), time.time() - t))
        
        #return 0
        return index
    
    
    def QuaterniontoRotationMatrix(self, q):
        ''' Calculates the Rotation Matrix from Quaternion
        s is the real part
        x, y, z  are the complex elements
        https://www.uni-koblenz.de/~cg/veranst/ws0001/sem/Bartz.pdf Chap. 1.2.6
        '''
        x, y, z, s = q
        
        # Precalculate repeatedly used values.
        x2 = x**2
        xy = x * y
        xz = x * z
        
        y2 = y**2
        yz = y * z
        
        z2 = z**2
        
        sz = s * z
        sy = s * y
        sx = s * x
        
        # Calculate rotation matrix.
        R11 = 1 - 2.0 * (y2 + z2)
        R12 = 2.0 * (xy - sz)
        R13 = 2.0 * (xz + sy)
        
        R21 = 2.0 * (xy + sz)
        R22 = 1 - 2.0 * (x2 + z2)
        R23 = 2.0 * (yz - sx)
        
        R31 = 2.0 * (xz - sy)
        R32 = 2.0 * (yz + sx)
        R33 = 1 - 2.0 * (x2 + y2)
        
        return np.matrix([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
    
    
    #def get_light_state(self, light):
    def get_light_state(self, light_pos_x, light_pos_y, light_pos_z):
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
        
        # Point_in_world (Point): 3D location of a point in the world.
        focal_len_x = self.config['camera_info']['focal_length_x']
        focal_len_y = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        
        # Get transform between pose of camera and world frame.
        rotation = None
        transform = None
        
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            transform, rotation = self.listener.lookupTransform("/base_link", "/world", now)
        
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Error, to find camera to map transform")
        
        if rotation == None or transform == None:
            visible = False
            x = -1
            y = -1
        
        # Use rotation and transform to calculate 2D position of light in image.
        light_pos_z = 1 # Doesn't work with correct z values.
        world_point = np.array([light_pos_x, light_pos_y, light_pos_z]).reshape(1, 3, 1)
        
        camera_mat = np.matrix([[focal_len_x, 0, image_width / 2],
                               [0, focal_len_y, image_height / 2],
                               [0, 0, 1]])
        dist_coeff = None
        
        # 4x1 -> quaternion to rotation matrix at z-axis.
        rot_vec, _ = cv2.Rodrigues(self.QuaterniontoRotationMatrix(rot))
        ret, _ = cv2.projectPoints(world_point, rot_vec, np.array(trans).reshape(3, 1), camera_mat, dist_coeff)
        
        # Unpack values and return.
        ret = ret.reshape(2,)
        
        # For some reason x & y are swapped.
        x = int(round(ret[1]))
        y = int(round(ret[0]))
        
        visible = False
        if x >= 0 and x < image_width and y >= 0 and y <= image_height:
            visible = True
        
        # Show car image.
        if DISPLAY_CAMERA:
            image_tmp = np.copy(cv_image)
            # Draw a circle.
            cv2.circle(image_tmp, (x, y), 20, (255, 0, 0), thickness = 2)
            cv2.imshow('image', image_tmp)
            cv2.waitKey(1)
        
        state = TrafficLight.UNKNOWN
        if visible:
            # Get classification.
            state = self.light_classifier.get_classification(cv_image)
        
        return state
    
    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        
        Returns:
            #int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #light = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection.
        #stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.position.x, self.pose.position.y)
            
            #TODO find the closest visible traffic light (if one exists)
            light_pos = None
            if car_position > 0:
                light_positions = self.config['light_positions']
                #light_positions = self.config['manual_light_positions']
                light_wp = sys.maxsize
                
                # Only one complete circle, no minimum distance considered, yet.
                for i in range(0, len(light_positions)):
                    index = self.get_closest_waypoint(float(light_positions[i][0]), float(light_positions[i][1]))
                    if index > waypoint_start_index and index < light_wp:
                        light_wp = index;
                        light_pos = light_positions[i]
                
                if light_pos:
                    #rospy.loginfo("Next traffic light ahead from waypoint " + str(car_position) + " is at position " + str(light_pos) + " at waypoint " + str(light_waypoint))
                    state = TrafficLight.UNKNOWN
                    if rospy.get_param('~use_classifier', False):
                        state = self.get_light_state(light_pos[0], light_pos[1], light_pos[2] if len(light_pos) >= 3 else 0.)
                    else:
                        for light in self.lights:
                            ''' If position of the light from the yaml file and one roperted via
                                /vehicle/traffic_lights differs only within 30 m consider them as same
                            '''
                            
                            euclidian_dist = math.sqrt((light.pose.pose.position.x - light_pos[0])**2 + (light.pose.pose.position.y - light_pos[1])**2)
                            
                            if euclidian_dist < 30:
                                #state = self.get_light_state(light.pose.pose.position.x, light.pose.pose.position.y, light.pose.pose.position.z)
                                state = light.state
                    
                    return light_waypoint, state
        
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
