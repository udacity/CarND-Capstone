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
import pprint             #format data structures into strings, for logging
from numpy import asarray
from scipy import spatial #supports data structure for looking up nearest point (essentially a binary search tree)
STATE_COUNT_THRESHOLD = 3
DETECTOR_ENABLED      = True   #Set True to use our actual traffic light detector instead of the message data
DEBUG_MODE            = True  #Switch for whether debug messages are printed.  Unless agotterba sets this to true, he doesn't get debug messages even in the tl_detector log file

class TLDetector(object):
    def __init__(self):

        if(DEBUG_MODE):
            rospy.init_node('tl_detector',log_level=rospy.DEBUG)
            rospy.logwarn("tl_detector: debug mode enabled.  Disable for PR; submission")
        else:
            rospy.init_node('tl_detector')

        if(not DETECTOR_ENABLED):
            rospy.logwarn("tl_detector: Detector is disabled; will use data from simulator.  set DETECTOR_ENABLED = True for submission")
            
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1,buff_size=2**24)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        image_size = (self.config['camera_info']['image_width'],self.config['camera_info']['image_height'])
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.img_count = 0

        self.wp_kdtree = None #tree to store waypoints, using scipy.spatial
        self.num_wp    = 0    #number of waypoints received from base_waypoints
        self.tl_list   = None #list to store traffic lights in waypoint order
        self.light_classifier = None #only declare if DETECTOR_ENABLED, since lots of work is done during initialization
        if (DETECTOR_ENABLED):
            self.light_classifier = TLClassifier(image_size,debug=rospy.logdebug,info=rospy.loginfo,warn=rospy.logwarn,error=rospy.logerr)

        self.ready_classifier = True
        self.ready_image_cb   = True

        rospy.spin()

    def pplog(self,level,data): #convert data structure to string from prettyPrinter and send to rospy.log<level>
        ppstring = pprint.pformat(data,indent=4)
        if (level == 'debug'):
            rospy.logdebug("   -- begin ppdata debug: --")
            rospy.logdebug(ppstring)
            rospy.logdebug("   -- end ppdata debug --")
            return
        if (level == 'info'):
            rospy.loginfo("   -- begin ppdata info: --")
            rospy.loginfo(ppstring)
            rospy.loginfo("   -- end ppdata info --")
            return
        if (level == 'warn'):
            rospy.logwarn("   -- begin ppdata warn: --")
            rospy.logwarn(ppstring)
            rospy.logwarn("   -- end ppdata warn --")
            return
        if (level == 'err'):
            rospy.logerr("   -- begin ppdata err: --")
            rospy.logerr(ppstring)
            rospy.logerr("   -- end ppdata err --")
            return
        if (level == 'fatal'):
            rospy.logfatal("   -- begin ppdata fatal: --")
            rospy.logfatal(ppstring)
            rospy.logfatal("   -- end ppdata fatal --")
            return
        rospy.logwarn("tl_detector: pplog received unrecognized level: %s",level)
        return


    
    def pose_cb(self, msg):
        self.pose = msg
        
        #rospy.logdebug("tl_detector: pose_cb received current pose:")
        #self.pplog('debug',self.pose)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        
        #rospy.logdebug("tl_detector: waypoints_cb received base waypoints:")
        #self.pplog('debug',self.waypoints)

        wp_coords = []
        for waypoint in self.waypoints.waypoints:
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            wp_coords.append((wp_x,wp_y))
            self.num_wp += 1
            
        self.wp_kdtree = spatial.KDTree(asarray(wp_coords))
        #tree_data = self.wp_kdtree.data
        #rospy.logdebug("wp_kdtree populated with data:")
        #rospy.logdebug(tree_data)
        
    def traffic_cb(self, msg):
        self.lights = msg
        
        #rospy.logdebug("tl_detector: traffic_cb received traffic lights:")
        #self.pplog('debug',self.lights)

        if (self.tl_list is None and self.wp_kdtree is not None): #we've received waypoints, but haven't initialized tl_list
            stop_line_positions = self.config['stop_line_positions']
            self.tl_list = []
            i = 0
            for tl in self.lights.lights:
                tl_hash = {}
                tl_hash['light_x'] = tl.pose.pose.position.x
                tl_hash['light_y'] = tl.pose.pose.position.y
                tl_hash['light_z'] = tl.pose.pose.position.z
                #Do we need orientation too?  only z and w have non-zero values
                tl_hash['stop_x']  = stop_line_positions[i][0]
                tl_hash['stop_y']  = stop_line_positions[i][1]
                tl_dist,tl_hash['wp'] = self.wp_kdtree.query(asarray((tl_hash['stop_x'],tl_hash['stop_y'])))
                tl_hash['state'] = TrafficLight.UNKNOWN
                self.tl_list.append(tl_hash)
                i += 1

            self.tl_list = sorted(self.tl_list, key=lambda k: k['wp']) #sort list by waypoint index
            rospy.logdebug("tl_detector: traffic_cb created tl_list")
            self.pplog('debug',self.tl_list)

        # This section updates traffic light state from data in msg;
        #    duplicates some code so that it can be easily disabled with DETECTOR_ENABLED
        #    Also enable when in DEBUG_MODE, to compare inference against ground truth
        if ((DEBUG_MODE or not DETECTOR_ENABLED) and self.tl_list is not None):
            stop_line_positions = self.config['stop_line_positions']
            state_tl_list = []
            i = 0
            for tl in self.lights.lights:
                state_tl_hash = {}
                stop_x = stop_line_positions[i][0]
                stop_y = stop_line_positions[i][1]
                state_tl_dist,state_tl_hash['wp'] = self.wp_kdtree.query(asarray((stop_x,stop_y)))
                state_tl_hash['state'] = tl.state
                state_tl_list.append(state_tl_hash)

            state_tl_list = sorted(state_tl_list, key=lambda k: k['wp']) #sort list by waypoint index

            for tl_hash,state_tl_hash in zip (self.tl_list,state_tl_list): #as locations don't change, sorting by nearest waypoint will generate same order
                tl_hash['state'] = state_tl_hash['state']

            #rospy.logdebug("tl_detector: traffic_cb updated states in tl_list:")
            #self.pplog('debug',self.tl_list)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.logdebug("tl_detector: entered image_cb")
        self.state_count += 1
        self.img_count += 1
        if not self.ready_image_cb:
            rospy.logdebug("tl_detector: image_cb received image %d, but is not ready",self.img_count)
            return

        self.ready_image_cb = False
        self.has_image = True
        self.camera_image = msg
        image_num = self.img_count
        rospy.logdebug("tl_detector: image_cb processing image %d",image_num)
        light_wp, state = self.process_traffic_lights(image_num)

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
            #report light if red or yellow, so we don't freak out or blow the stop line when it suddenly turns red
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.logdebug("tl_detector: from image %d (vs. current image %d), published waypoint of next red light's stop line: %d",image_num,self.img_count,light_wp)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rospy.logdebug("tl_detector: from image %d (vs. current image %d), published waypoint of prevously reported red light's stop line: %d",image_num,self.img_count,self.last_wp)

        self.ready_image_cb = True
            
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #DONE: implement
        #rospy.logdebug("entered get_closest_waypoint")
        pose_x = pose.position.x
        pose_y = pose.position.y
        #rospy.logdebug("  pose_x,pose_y are %f,%f",pose_x,pose_y)

        #initial brute force, but kdtree is a more optimized solution to this problem
        # index = 0
        # min_dist = 1.0e9 #large initial value; the moon is 4e8 meters away.
        # min_dist_index = -1
        # close_wp_x = 0.0 #for logging only
        # close_wp_y = 0.0
        # for waypoint in self.waypoints.waypoints:
            
        #     wp_x = waypoint.pose.pose.position.x
        #     wp_y = waypoint.pose.pose.position.y
        #     dist = ((pose_x - wp_x)**2 + (pose_y - wp_y)**2)**0.5
        #     if (dist < min_dist):
        #         min_dist = dist
        #         min_dist_index = index
        #         close_wp_x = wp_x
        #         close_wp_y = wp_y
        #     index += 1

        # rospy.logdebug ("found waypoint %d is closest to position %f,%f :",min_dist_index, pose_x,pose_y)
        # rospy.logdebug ("  waypoint %d at position %f,%f has distance %f",min_dist_index,close_wp_x,close_wp_y,min_dist)

        kd_dist,kd_index = self.wp_kdtree.query(asarray((pose_x,pose_y)))
        kd_point = self.wp_kdtree.data[kd_index]
        #rospy.logdebug("wp_kdtree returns index %d with coordinates %f,%f; distance %f",kd_index,kd_point[0],kd_point[1],kd_dist)
        
        return kd_index

    def get_light_state(self, light,image_num):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logdebug("tl_detector: entered get_light_state")
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        contemp_state = self.tl_list[light]['state']
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if(DETECTOR_ENABLED):
            #Get classification
            #return self.light_classifier.get_classification(cv_image)
            if(self.ready_classifier == True):
                self.ready_classifier = False
                classifier_state =  self.light_classifier.get_classification(cv_image)
                self.ready_classifier = True
            else:
                rospy.logdebug("tl_detector: get_light_state: skipping inference for image %d because the classifier isn't ready yet.  Returning previous value %d",image_num,self.state)
                classifier_state = self.state
            rospy.logdebug("tl_detector: get_light_state: classifer for image %d returned state %d, vs contemp. state %d and current sim state %d",image_num,classifier_state,contemp_state,self.tl_list[light]['state'])
            #return self.tl_list[light]['state']
            return classifier_state
        else:
            rospy.logdebug("tl_detector: get_light_state: detector not enabled; returning simulator light state %d"%self.tl_list[light]['state'])
            return self.tl_list[light]['state']

    def process_traffic_lights(self,image_num):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logdebug("tl_detector: entered process_traffic_lights")
        light = None
        light_wp = None

        # List of positions that correspond to the line to stop in front of for a given intersection.  already saved this to tl_list
        #stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            rospy.logwarn("tl_detector: process_traffic_lights not calling get_closest_waypoint because pose is false")
            rospy.logwarn("  pose:")
            self.pplog('warn',self.pose)
            
        #DONE: find the closest visible traffic light (if one exists)
        #doesn't test for visibility.  just reports first upcoming stop line.  Will some minimum distance (equiv to the 200 waypoints) be sufficent?
        wps_to_closest_tl = 1e6
        i = 0
        for tl_hash in self.tl_list: #check all lights to find closest. 
            wps_to_tl = tl_hash['wp'] - car_position
            if (wps_to_tl < 0):   # we've wrapped around waypoint list to beginning
                wps_to_tl += self.num_wp
            if (wps_to_tl < wps_to_closest_tl):
                wps_to_closest_tl = wps_to_tl
                light = i
                light_wp = tl_hash['wp']

        if light is not None:
            state = self.get_light_state(light,image_num)
            return light_wp, state

        rospy.logwarn("tl_detector: process_traffic_lights did not find any light that was next.  this shouldn't happen")
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        rospy.logdebug('tl_detector: Declaring TLDetector')
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('tl_detector: Could not start traffic node.')
