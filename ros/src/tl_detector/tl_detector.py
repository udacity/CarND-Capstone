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
import yaml
import sys
import os
import math

STATE_COUNT_THRESHOLD = 3
DISTANCE_LIMIT = 100

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

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
	#rospy.loginfo(os.path.join(os.getcwd(), "light_classification/gc_classifier.pkl"))
        self.light_classifier = TLClassifier(os.path.join(os.getcwd(), "light_classification/gc_classifier_v1_p27_est.pkl"))
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.tl_min_distance = 10000

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def euclidean_distance(self,refx,refy,refz,curx,cury,curz):
        distance = math.sqrt((curx - refx) ** 2 + (cury - refy) ** 2 + (curz - refz) ** 2)
	return distance

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if (self.waypoints == None):
            return None
        else:
            waypt = self.waypoints.waypoints

	# Create variables for nearest distance and neighbour
        closest_wp_index = None
        min_distance = sys.maxsize

        # Find nearest points
        for i in range(len(waypt)):
            cur_wp_pos = waypt[i].pose.pose.position
            dist  = self.euclidean_distance(pose.position.x,pose.position.y,pose.position.z,cur_wp_pos.x,cur_wp_pos.y,cur_wp_pos.z)
            if dist < min_distance:
                closest_wp_index = i
                min_distance = dist

        return closest_wp_index

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

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        min_distance = sys.maxsize
        light_wp_idx = None

        # update waypoint position closest to the car_position
        if (self.waypoints == None) or (car_position == None):
            return -1, TrafficLight.UNKNOWN
        else:
            car_pose = self.waypoints.waypoints[car_position].pose.pose.position

        # Find the nearest traffic lights
        num_light_idx = len(self.lights)
        for i in range(num_light_idx):
            lt_pos = self.lights[i].pose.pose.position
            dist  = self.euclidean_distance(car_pose.x,car_pose.y,car_pose.z,lt_pos.x,lt_pos.y,lt_pos.z)
            if dist < min_distance:
	        light_wp_idx 	= i
                min_distance    = dist
        
        #rospy.loginfo("car_position: %d", car_position)
        stop_wp_idx = None        
        min_dist = sys.maxsize
        close_light_idx  =  self.get_closest_waypoint(self.lights[light_wp_idx].pose.pose)

        if ((light_wp_idx is not None) and (close_light_idx > (car_position + 1))):
            light = self.lights[light_wp_idx].pose.pose.position
            state = self.get_light_state(light)

            #find the stop line waypoint to closest traffic light
            for i in range(0, len(stop_line_positions)):
                stop_line_pos = PoseStamped()
                stop_line_pos.pose.position.x = stop_line_positions[i][0]
                stop_line_pos.pose.position.y = stop_line_positions[i][1]
                stop_line_pos.pose.position.z = 0
                closest_wp_idx  =  self.get_closest_waypoint(stop_line_pos.pose)
                stop_lt_pos     = self.waypoints.waypoints[closest_wp_idx].pose.pose.position
                dist            = self.euclidean_distance(stop_lt_pos.x, stop_lt_pos.y, stop_lt_pos.z, light.x, light.y,0)
                 
                if (dist < min_dist) and (closest_wp_idx > (car_position + 1)):
                    stop_wp_idx = closest_wp_idx
                    min_dist    = dist

            if stop_wp_idx is not None:
                #rospy.loginfo("Traffic distance: %d  %d %d", min_dist, stop_wp_idx, car_position)
                
                # only update traffic light if min distance is within distance limit
                stop_line_pos = self.waypoints.waypoints[stop_wp_idx].pose.pose.position
                self.tl_min_distance = self.euclidean_distance(car_pose.x,car_pose.y,car_pose.z,stop_line_pos.x,stop_line_pos.y,stop_line_pos.z)
                if (self.tl_min_distance < DISTANCE_LIMIT):
		    if state == 0:
		       state_str = "RED"
		    elif state == 1:
		       state_str = "YELLOW"
		    else:
		       state_str = "GREEN"
                    rospy.loginfo("curr_wp_idx = %d, stop_wp_idx = %d, state = %s",car_position, stop_wp_idx, state_str)
                    return stop_wp_idx, state
                else:
                    return -1, TrafficLight.UNKNOWN
	else:
        	return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
