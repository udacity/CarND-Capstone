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
import os

label = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

	self.has_image = False

        # First argument: Corresponding waypoint id
        # Second argument: corrsponding light id
        self.idx_of_stop_line = None


        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

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

        self.light_classifier = TLClassifier(os.getcwd()+'/../../../training/Simulator')
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # Precalculate corresponding waypoint and traffic light index of stop line
        while self.idx_of_stop_line is None:
            if self.waypoints is not None and self.lights is not None:
                self.idx_of_stop_line = []
                stop_line_positions = self.config['stop_line_positions']
                for i in xrange(len(stop_line_positions)):
                    pose = Pose()
                    pose.position.x = stop_line_positions[i][0]
                    pose.position.y = stop_line_positions[i][1]
                    wp_id = self.get_closest_waypoint(pose)

                    min_dist_2 = 1e9
                    light_id = 0
                    for j in xrange(len(self.lights)):
                        light_x = self.lights[j].pose.pose.position.x
                        light_y = self.lights[j].pose.pose.position.y
                        dist_2 = math.pow(pose.position.x - light_x, 2) + math.pow(pose.position.y - light_y, 2)
                        if dist_2 < min_dist_2:
                            min_dist_2 = dist_2
                            light_id = j

                    self.idx_of_stop_line.append([wp_id, light_id])

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        self.position = self.pose.position
        self.orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w])
        self.theta = euler[2]
        if self.light_classifier is not None:
            if self.light_classifier.predict is None:
                print "NOT MOVING!   Initializing TRAFFIC LIGHT DETECTOR....", self.has_image
        else:
            print "WARNING!   NO TRAFFIC LIGHT DETECTOR...."

    def waypoints_cb(self, msg):
        # make our own copy of the waypoints - they are static and do not change
        if self.waypoints is None:
            self.waypoints = []
            for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)
        self.wlen = len(self.waypoints)

        # only get it once - reduce resource consumption
        self.sub2.unregister()
        self.sub2 = None

        

    def traffic_cb(self, msg):
        self.lights = msg.lights

        #if self.wpidx_of_lights is None and self.waypoints

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
        min_dist_2 = 1e9
        ret_index = 0
        for i in xrange(len(self.waypoints)):
            wp_x = self.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints[i].pose.pose.position.y
            pose_x = pose.position.x
            pose_y = pose.position.y
            dist_2 = math.pow(wp_x - pose_x, 2) + math.pow(wp_y - pose_y, 2)
            if dist_2 < min_dist_2:
                min_dist_2 = dist_2
                ret_index = i

        return ret_index

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
        #return light.state
        

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        visible_distance = 50.
        visible_angle = 50. * math.pi/180.\

        if self.pose is not None and self.waypoints is not None and self.idx_of_stop_line is not None:
            car_x = self.pose.position.x
            car_y = self.pose.position.y
            car_theta = 2.*math.acos(self.pose.orientation.w)
            car_theta %= (2.*math.pi)
            if car_theta > math.pi:
                car_theta -= 2.*math.pi

            for k in xrange(len(self.idx_of_stop_line)):
                i = self.idx_of_stop_line[k][0]
                stop_x = self.waypoints[i].pose.pose.position.x
                stop_y = self.waypoints[i].pose.pose.position.y
                diff_x = stop_x - car_x
                diff_y = stop_y - car_y

                diff_theta = math.atan2(diff_y, diff_x)
                diff_theta %= (2.*math.pi)
                if diff_theta > math.pi:
                    diff_theta -= 2.*math.pi

                dist_2 = math.pow(diff_x, 2) + math.pow(diff_y, 2)

                if abs(car_theta - diff_theta) < visible_angle and dist_2 < math.pow(visible_distance, 2):
                    # Determine there should be traffic light ahead of the car.
                    light_wp = i
                    light_id = self.idx_of_stop_line[k][1]
		    print ("light id:",light_id)
                    state = self.get_light_state(self.lights[light_id])
		    print("state:",state)
                    return light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
