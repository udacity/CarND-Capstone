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
import copy
import sys

import datetime
import pdb
from math import sin,cos

# Set to true to save images from camera to png files
# Used to zoom test mapping of 3D world coordinates to 
# image plane.
# If true, requires keyboard input at each function call
image_capture_mode = False
img_dir = 'test_img'

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Custom attributes
        self.last_car_wp = None
        self.line_pos_wp = []


        
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
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # Only update once since this is static
        if waypoints:
            print('Updating with %i waypoints'%len(waypoints.waypoints))
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
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, last_ind = None):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            last_ind: Index of last known waypoint. If not None,
                only search waypoints around the last_ind

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement


        pos = pose.position # position of car
        if self.waypoints is None:
            print('no waypoints, returning None')
            return None
        else:
            # If last index is supplied, only search for waypoints
            # around that region
            if last_ind: 
                start_ind = max(last_ind - 200,0)
                end_ind = min(last_ind + 200,len(self.waypoints.waypoints))
                search_wp = self.waypoints.waypoints[start_ind:end_ind]
            # Else search all waypoints (10,000 of them)
            else:
                start_ind = 0
                search_wp = self.waypoints.waypoints
            
            # Search the waypoints for the closest distance to pose
            closest_dist = 10**6
            closest_ind = 0
            for i,waypoint in enumerate(search_wp):
                way = waypoint.pose.pose.position
                dist = ((pos.x - way.x)**2 + (pos.y - way.y)**2)**0.50
                if dist < closest_dist:
                    closest_ind = i + start_ind
                    closest_dist = dist
            return closest_ind


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        # Focal length in config of unknown units. Normally given in 
        # thousands of pixels but the number is order 1
        # temporarily, just assign it some reasonable number
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        fx = 1000
        fy = 600

        # Overwrite image size until update
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        image_width = 800
        image_height = 600


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
        # Convert light position to local car coords
        quaternion = (self.pose.pose.orientation.x
                      ,self.pose.pose.orientation.y
                      ,self.pose.pose.orientation.z
                      ,self.pose.pose.orientation.w)

        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)

        shift_x = point_in_world.x - self.pose.pose.position.x
        shift_y = point_in_world.y - self.pose.pose.position.y

        car_x = shift_x * cos(-yaw) - shift_y * sin(-yaw)
        car_y = shift_x * sin(-yaw) + shift_y * cos(-yaw)
        #cam_height = self.pose.pose.position.z
        cam_height = 1.5 #experimental values
        car_z = point_in_world.z - cam_height



        # Calculate position of point in world in image

        # x is the col number
        delta_x = car_y * fx / car_x
        x = int(image_width/2 - delta_x)


        # v is the row number
        delta_y = car_z * fy / car_x
        y = int(image_height/2 - delta_y)

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For debugging, return the provided true value
        # Won't be available in the real run.
        # In future, will need to capture images and use machine vision
        # to determine the state
        use_true_value = True
        if use_true_value:
            return light.state



        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # u,v are the x,y in the image plane
        u,v  = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        # Image capture
        if image_capture_mode:

            # Uncomment below to wait until pressing "enter" to take a pic
            # allows moving the car to a desirable position for the pic
            '''
            shutter_msg = 'Press enter to take picture and continue'
            if sys.version_info > (3,):
                input(shutter_msg)
            else:
                raw_input(shutter_msg)
            '''
            
            # Write text
            y0 = 50
            dy = 20
            for i,line in enumerate(str(self.pose).split('\n')):
                y = y0 + i*dy
                cv2.putText(cv_image,line,(50,y)
                    ,cv2.FONT_HERSHEY_PLAIN,1,255)

            # Draw position of light
        
            cv2.line(cv_image,(u-100,v),(u+100,v),(0,0,255),5)
            cv2.line(cv_image,(u,v-100),(u,v+100),(0,0,255),5)
            
            # Specify filename and write image to it
            img_path = '%s/%s.png'%(img_dir,datetime.datetime.now())
            cv2.imwrite(img_path,cv_image)
            print('image written to:',img_path)


        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # If care position is known, get index of closest waypoint
        if(self.pose):
            car_wp = self.get_closest_waypoint(self.pose.pose,self.last_car_wp)
            self.last_car_wp = car_wp
            if not car_wp:
                print('car waypoint could not be found')
                return -1, TrafficLight.UNKNOWN
        else:
            print('self.pose is emtpy')
            return -1, TrafficLight.UNKNOWN

        #TODO find the closest visible traffic light (if one exists)

        # Get waypoints of all traffic light lines if not already done
        if self.line_pos_wp == []:
            stop_line_positions = self.config['stop_line_positions']
            self.line_list = []
            for line_pos in stop_line_positions:
                # Make deepcopy bc I don't know how to make one
                # from scratch
                this_line = copy.deepcopy(self.pose)
                this_line.pose.position.x = line_pos[0]
                this_line.pose.position.y = line_pos[1]

                # Get closest waypoint to this line
                this_line_wp = self.get_closest_waypoint(this_line.pose)
                self.line_pos_wp.append(this_line_wp)
    
                # Make deep copy bc to add to list
                # Otherwise this_line would be altered inside list
                self.line_list.append(copy.deepcopy(this_line))

        # Get closest waypoint (of foward waypoints) to the position waypoint
        delta_wp = [wp-car_wp for wp in self.line_pos_wp]
        min_delta_wp = min(d for d in delta_wp if d>=0)
        line_wp_ind = delta_wp.index(min_delta_wp)

        # Assign a light waypoint only if within visible distance
        line = None
        visible_num_wp = 150
        if min_delta_wp < visible_num_wp:
            line = self.line_list[line_wp_ind]
            line_wp = self.line_pos_wp[line_wp_ind]

        if line:
            state = self.get_light_state(self.lights[line_wp_ind])
            print('')
            print('Msg from tl_detector.py')
            print('light detected')
            print('car waypoint: ',car_wp)
            print('line_waypoint: ',line_wp, state)
            return line_wp, state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
