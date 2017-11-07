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
import math
import copy

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = []
        self.camera_image = None
        self.lights = []
        self.stop_lin_pos = None
        self.count =0
      
      
       ###### replace the subscriber with waiting for msg       
        wp = rospy.wait_for_message('/base_waypoints', Lane)
        
        #if not self.base_waypoints:
        self.base_waypoints = wp.waypoints
        self.datasize = len(self.base_waypoints)
        #rospy.logwarn('Got the base points in Tl detec %s', self.datasize)        
        
        b_xcor = []
        b_ycor = []
        for pt in self.base_waypoints:
            b_xcor.append(pt.pose.pose.position.x)
            b_ycor.append(pt.pose.pose.position.y)
        # Make a KD tree for distance measurement    
        self.tree= KDTree(zip(b_xcor, b_ycor))
 
        ###################################
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #aub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
       
      
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        # get list of stop line waypoints form yaml
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        # list contains waypoints for stop line
        #self.stop_lin_pos = self.config['stop_line_positions']

        # publish the traffic waypoint
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()
        

    def pose_cb(self, msg):
        self.pose = msg.pose


#    def waypoints_cb(self, waypoints):    
#        self.waypoints = waypoints

    def traffic_cb(self, msg):  
        self.lights = msg.lights      #Traffic light array


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        if not self.base_waypoints: 
            #rospy.logwarn('No self.waypoints \n\n')
            pass
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:   # current state different than new state
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state != TrafficLight.RED: # Not Red
                light_wp = -light_wp                
        
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to n position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        
        _,idx = self.tree.query((pose.position.x,pose.position.y))        
        
        
        return idx

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
        TL_array = len(self.lights)
        stop_line_idx = float('inf')
        min_dist = float('inf')
        closest_light_index = float('inf')
        
        
        #rospy.logwarn('inside process traffic lights')
        
        '''
        Three step process: First get index of Ego location, then find index of the 
        stop line closest to Ego. Lastly, use that stop_line index to find the
        closest traffic light index. That index will give us the traffic light from the TL array
        which is  closest to us and then is used to find colour.  
        '''        
        
        # get waypoint index closest to car
        # this index can be behind the cr or head of the car
        if(self.pose):
            car_position_idx = self.get_closest_waypoint(self.pose)
            
        
            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']
            
            # get index of the closest stop line    
            for stop_line in stop_line_positions:
                pt = PoseStamped()
                pt.pose.position.x = stop_line[0]
                pt.pose.position.y = stop_line[1]
                pt.pose.position.z = 0
                # get index of the closest line
                idx = self.get_closest_waypoint(pt.pose)
               
               # Check if idx is ahead of us  and is closest
                if (idx > car_position_idx) and (idx < stop_line_idx):
                    stop_line_idx = idx         # if yes, this is our closest stop line 
        
        #rospy.logwarn('Stop line idx is %s ', stop_line_idx)  
        if stop_line_idx == float('inf') or stop_line_idx >= self.datasize:  
            return -1, TrafficLight.UNKNOWN          
          
        
        
        
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        #TODO find the  visible TL closest to stop_line_index by looping through all lights
        for q in range(TL_array):
            #rospy.logwarn('idx is %s total pts %s   array %s' , q, len(self.base_waypoints),TL_array)   
            dist = dl(self.base_waypoints[stop_line_idx].pose.pose.position, self.lights[q].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_light_index = q          # closest light 
        # Error checking        
        if closest_light_index == float('inf'):  
            return -1, TrafficLight.UNKNOWN
            
        light = self.lights[closest_light_index]
  
        if ((self.count%5) == 0):
            #rospy.logwarn('differ between car and stop  %f m', dl(self.base_waypoints[stop_line_idx].pose.pose.position, self.pose.position))
            rospy.logwarn('Light # %s coming in %f m \n\n\n', closest_light_index,dl(self.lights[closest_light_index].pose.pose.position,self.pose.position))

        self.count += 1
        if light:
            #TODO replace the ground truth by actual function 
            state = light.state         
            #state = self.get_light_state(light)   
            return stop_line_idx, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
