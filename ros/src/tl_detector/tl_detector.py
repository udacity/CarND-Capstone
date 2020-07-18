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
import csv
from datetime import datetime
import os


STATE_COUNT_THRESHOLD = 3

# configuration for saving training data from simulator
SAVE_TRAINING_IMAGE = False
SLOW_MOTION_AT_LIGHT = True
MAX_NUM_IMG_SAVE = 10
SAVE_LOCATION = "./light_classification/sim_img/"

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        
        # for saving images
        self.num_image_saved = [0] * 4; # keep track of number of image stored for each light state (current run)
        self.image_saver_cooldown = 0;

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

        # Publisher for red light waypoint position
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        if SAVE_TRAINING_IMAGE:
            if not os.path.exists(SAVE_LOCATION):
                os.makedirs(SAVE_LOCATION)
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_cb(self, msg):
        """Gets current pose of the vehicle"""
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """Stores base waypoints as a KDTree"""
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        """Gets traffic light positions"""
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
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #just for testing
        return light.state
        '''
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        Get classification
        return self.light_classifier.get_classification(cv_image)#
        '''

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
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        # Find the closest visible traffic light (if one exists)
        diff = len(self.waypoints.waypoints)

        # Loop through traffic light positions
        stop_line_immediate_behind = False # flag if just past stop line - the previous light may still be in sight!
        
        for i, light in enumerate(self.lights):
            # Get stop line waypoint index
            line = stop_line_positions[i]
            temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

            # Find closest line
            d = temp_wp_idx - car_position
            
            if d < 0 and d > -25:
                # This flag is only used during training data collecting
                # Often when car crosses stop line, closest_light jumps to next light far ahead
                # But the stopline of the light we just passed several waypoints ago is still within the sight of the camera
                # An example of conflict: ground truth says there is no light present (within 300 waypoints ahead) but camera still sees a greenlight that we just passed
                stop_line_immediate_behind = True
            
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                line_wp_idx = temp_wp_idx

        # if found a closest light and is 300 waypoints in front of us
        if closest_light and diff < 300:
            # get state of traffic light
            state = self.get_light_state(closest_light)
            # return line waypoint index & traffic light state
            
            if SAVE_TRAINING_IMAGE and not stop_line_immediate_behind:
                state_truth = closest_light.state # make sure use ground truth
                self.save_training_img(state_truth,diff)
                
                if SLOW_MOTION_AT_LIGHT:
                    # Trick the car to slow down when obtaining training data by broadcasting a imaginary red light stop line ahead
                    return (car_position + 10), 0
            
            return line_wp_idx, state

        else:
            # no upcoming traffic light was found
            if SAVE_TRAINING_IMAGE and not stop_line_immediate_behind:
                state_truth = 3 # new class 3 'no traffic ligh present
                self.save_training_img(state_truth,diff)
            
            return -1, TrafficLight.UNKNOWN

    def save_training_img(self,state_truth,dist_next_light):
        """
        call this function when a valid camera image with ground truth is available for saving
        This has an internal cool-down to prevent dumping same image too often
        """
        if self.num_image_saved == [MAX_NUM_IMG_SAVE]*4 :
            rospy.loginfo("Max amount of training image (%d)for all class achieved." % MAX_NUM_IMG_SAVE)
            return
        
        if self.image_saver_cooldown == 0:
            # save image at 5 loop interval and up to X total for each state
            if self.num_image_saved[state_truth] < MAX_NUM_IMG_SAVE :
                if(self.has_image):
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                    except CvBridgeError, e:
                        rospy.loginfo(e)
                    else:
                        (dt, micro) = datetime.utcnow().strftime('%Y%m%d%H%M%S.%f').split('.')
                        dt = "%s%03d" % (dt, int(micro) / 1000) 
                                                                
                        fname = "_state_%d_image_%s.jpeg" % (state_truth,dt)
                                
                        cv2.imwrite(SAVE_LOCATION+fname, cv_image)
                        self.num_image_saved[state_truth] += 1
                        
                        if state_truth == 1:
                            #yellow is too rare - use less cooldown
                            self.image_saver_cooldown = 1
                        else:
                            self.image_saver_cooldown = 5   
                                
                        csv_file_name = SAVE_LOCATION+"sim_images.csv"

                        with open(csv_file_name,'a') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow([fname , state_truth, dist_next_light])
                        
                        if state_truth == 3:
                            rospy.loginfo("New image saved: Traffic light state 3. Next light is %d waypoints ahead"
                                  % (dist_next_light))
                        else:
                            rospy.loginfo("New image saved: Traffic light state %d at %d waypoints ahead" %
                                  (state_truth,dist_next_light))    
                        
                        rospy.loginfo("Total [%d,%d,%d,%d] saved this run" 
                              % (self.num_image_saved[0],self.num_image_saved[1],
                                 self.num_image_saved[2],self.num_image_saved[3]))
                        
        else:
            self.image_saver_cooldown -= 1
            self.image_saver_cooldown = max(0,self.image_saver_cooldown)
        
    
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
