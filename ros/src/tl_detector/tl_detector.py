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
import math
import numpy as np
from scipy import spatial
import os
from PIL import Image as PIL_Image

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        #rospy.init_node('tl_detector', log_level=rospy.DEBUG)
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.kdtree_lights = None
        self.kdtree_waypoints = None
        self.lights_wps = None
        self.kdtree_light_wps = None
        self.save_image_seq = 0

        self.camera_image = None
        self.lights = []
        #when run with simulator, counting how many times that the detected state doesn't match the simulator's
        self.light_state_wrong = 0

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
	self.bypass_light_classify = self.config["bypass_light_classify"]
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.create_dump_dir("red")
        self.create_dump_dir("green")
        self.create_dump_dir("yellow")
        self.create_dump_dir("unknown")

	rospy.loginfo("tl_detector created")

        rospy.spin()

    def create_dump_dir(self, label):
        path = "./images/" + label
        if not os.path.exists(path):
            os.makedirs(path)


    def pose_cb(self, msg):
        self.pose = msg

    def build_waypoints_for_stoplines(self):
        if self.kdtree_light_wps is None :
            stop_line_positions = self.config['stop_line_positions']
            stop_points = [(p[0], p[1]) for p in stop_line_positions]
            wps_dists, wps_idcs = self.kdtree_waypoints.query(stop_points)
            self.lights_wps = wps_idcs

    #no matter which of waypoints_cb or traffic_cb is called first, always build the tree when waypoints, traffics are ready
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.waypoints_pts = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in waypoints.waypoints])
        self.kdtree_waypoints = spatial.KDTree(self.waypoints_pts)
        self.build_waypoints_for_stoplines()


    def traffic_cb(self, msg):
        self.lights = msg.lights
        #print (msg.lights)
        self.kdtree_lights = spatial.KDTree(np.array([ (l.pose.pose.position.x, l.pose.pose.position.y) for l in msg.lights]))

    def saveImage(self, img, state):
        if hasattr(img, 'encoding'):
            if img.encoding == '8UC3':
                img.encoding = "rgb8"
        else:
            img.encoding = 'rgb8'
        img = self.bridge.imgmsg_to_cv2(img, "rgb8")

        #image_data = cv2.resize(img, (224,224))
        image_data = img
        img= PIL_Image.fromarray(image_data, 'RGB')
        if state == TrafficLight.RED:
            img.save('./images/red/'+str(self.save_image_seq).zfill(5)+'.png', 'PNG')
            self.save_image_seq += 1
        if state == TrafficLight.YELLOW:
            img.save('./images/yellow/'+str(self.save_image_seq).zfill(5)+'.png', 'PNG')
            self.save_image_seq += 1
        elif state == TrafficLight.GREEN:
            img.save('./images/green/'+str(self.save_image_seq).zfill(5)+'.png', 'PNG')
            self.save_image_seq += 1
        else:
            img.save('./images/unknown/'+str(self.save_image_seq).zfill(5)+'.png', 'PNG')
            self.save_image_seq += 1
    

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
        #state encode: 0: red 1:yellow 2:green
        #rospy.loginfo ("state:{} self.state:{} last_state:{} count:{} isred:{}".format(state, self.state, self.last_state, self.state_count, state == TrafficLight.RED))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #rospy.loginfo("publish stable light : {} state count:{}".format(Int32(light_wp)), self.state_count)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #rospy.loginfo("publish last light : {}".format(Int32(self.last_wp)))
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
        if not self.waypoints or not pose:
            return -1

        """Using KDTree """
        cur_pose = [pose.position.x, pose.position.y]
        dist, idx = self.kdtree_waypoints.query(cur_pose)
        #rospy.logdebug("dist {}, idx {}".format(dist, idx))
        return idx

        """brute froce search version:
        def distance(pos1, pos2):
            return math.sqrt((pos1.position.x - pos2.position.x)**2 + 
                             (pos1.position.y - pos2.position.y)**2)
        if self.waypoints is None or len(self.waypoints) == 0: 
            return -1

        closest = 0
        min_dist = distance(pose, self.waypoints[0].pose.pose)
        for i in range(len(self.waypoints[1:])):
            dist = distance(pose, self.waypoints[i].pose.pose)
            if dist < min_dist:
                closest = i
                min_dist = dist
        return closest
        """

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

        rospy.logdebug("expected light state: {}".format(light.state))
        if self.bypass_light_classify:
            return light.state

        #Get classification
        state = self.light_classifier.get_classification(cv_image) 
        #self.saveImage(self.camera_image, state)
        if state != light.state and state != TrafficLight.UNKNOWN:
            self.light_state_wrong += 1
            if 200 > self.state_count > 100 :
                self.saveImage(self.camera_image, state)
            rospy.loginfo("light state wrong. Expected:{} Detected:{} state count:{}".format(light.state, state, self.state_count)) 
        return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.waypoints is None or self.pose is None:
            return -1, TrafficLight.UNKNOWN
		
        light = None
        light_wp = -1

        # List of positions that correspond to the line to stop in front of for a given intersection

        car_position = self.get_closest_waypoint(self.pose.pose)


        #TODO find the closest visible traffic light (if one exists)
        diff = len(self.waypoints)
        for wp, lgt in zip(self.lights_wps, self.lights):
            d =  wp - car_position

            if wp >= 0 and d >= 0 and d < diff:
                diff = d
                light = lgt
                light_wp = wp



        """ brute force search
        stop_line_positions = self.config['stop_line_positions']
        diff = len(self.waypoints)
        line_pos = Pose()
        for i, lgt in enumerate(self.lights):
            line_pos.position.x = stop_line_positions[i][0]
            line_pos.position.y = stop_line_positions[i][1]
            temp_wp_idx = self.get_closest_waypoint(line_pos)
            d = temp_wp_idx - car_position

            if temp_wp_idx >= 0 and d >= 0 and d < diff:
                diff = d
                light = lgt
                light_wp = temp_wp_idx
        """

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN





if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
