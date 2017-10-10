#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import Udacity (empty) traffic light classifier
from light_classification.tl_classifier import TLClassifier
# tf is related to ROS transforms, not to TensorFlow
import tf
import cv2
import yaml
import math

# the following 4 routines are
# cut-and-pasted from update_waypoint.py

# takes geometry_msgs/Point
# returns float
def point_dist_sq(a, b):
    dx = a.x-b.x
    dy = a.y-b.y
    dz = a.z-b.z
    return dx*dx+dy*dy+dz*dz

# takes geometry_msgs/Point
# returns float
def point_dist(a, b):
    return math.sqrt(point_dist_sq(a, b))

# takes styx_msgs/Waypoint
# returns geometry_msgs/Point
def waypoint_to_point(wp):
    point = wp.pose.pose.position
    return point

# takes styx_msgs/PoseStamp.pose
# returns geometry_msgs/Point
def pose_to_point(pose):
    point = pose.position
    return point

# takes x and y position (floats)
# returns geometry_msgs/Pose
def point_to_pose(x, y):
    pt = Point()
    pt.x = x
    pt.y = y
    pose = Pose()
    pose.position = pt
    return pose

STATE_COUNT_THRESHOLD = 5

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # Current pose (location and yaw, etc) of the car.
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # List of all the waypoints, containing the x,y,yaw values of each
        # waypoint.  Only needs to be received once, since the waypoints 
        # never change.
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        Udacity comment:
        /vehicle/traffic_lights provides you with the location of the traffic 
        light in 3D map space and helps you acquire an accurate ground truth 
        data source for the traffic light classifier by sending the current 
        color state of all traffic lights in the simulator. When testing on 
        the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        # In our code, the following information is published only when 
        # something # changes, such as a light changing state from green to 
        # yellow to red.
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # This topic carries the image seen by the car's camera.
        # This is the image that we have to analyze in order to
        # detect the state of the next traffic light.
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        # This is not part of the original Udacity code.  This
        # topic is published by our version of waypoint_updater.py.
        # It provides the id number of the next waypoint ahead of the car.
        sub7 = rospy.Subscriber('/next_waypoint', Int32, self.next_waypoint_cb)

        # Get the location of the file containing the "traffic light
        # configuration".  This parameter is set in the tl_detector/launch
        # directory, and it points to a file named either
        # tl_detector/sim_traffic_light_config.yaml or
        # tl_detector/site_traffic_light_config.yaml, depending on whether
        # we are using the simulator or running on the Udacity test track.
        config_string = rospy.get_param("/traffic_light_config")

        # Load the config file.  
        # The file contains the x,y position of the stop-line in front
        # of each traffic signal, and some information about the car's
        # camera.  It is generally believed (on slack) that the 
        # focal-length information is incorrect for the simulator, but 
        # correct for Udacity's real car.
        # We assume that the order of stop lines in the file corresponds
        # to the order of lights provided by the /vehicle/traffic_lights
        # topic.
        # Later in the code, we will compute a waypoint id from each of the
        # stop-line x,y positions, and append it to the x,y info.
        self.config = yaml.load(config_string)

        # This is the only thing that will be published as a result of all
        # our code: the waypoint id of the stop-line
        # at the next red light in front of the car.  If the next light 
        # is not red, or no lights are close enough to be visible, 
        # then -1*(waypoint_id) is published instead of the waypoint id.  
        # This topic 
        # is subscribed to by dbw_node.py, which uses the information 
        # to decide whether to brake or accelerate.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Contradicting the previous comment, a "targeted image" can
        # be published, but it is only used for diagnostics.  
        # Recall that the car sends us images from its camera. In
        # code further below, you will use the car's position, and the 
        # traffic signal location, to zoom in on this camera image,
        # in order to concentrate on the sub-area of the image where 
        # the traffic signal should be visible.  If you want, you can publish
        # this zoomed-in image (the "targeted image") so that you
        # can view it in rqt_image_view.
        self.targeted_image_pub = rospy.Publisher('/targeted_image', Image, queue_size=1)

        # CvBridge is provided by ROS as a bridge between ROS images
        # and OpenCV images
        self.bridge = CvBridge()

        # Defined in tl_detector/light_classification/tl_classifier.py.
        # This is where the classification work gets done.  It takes
        # an image as input, and returns a value telling whether the
        # traffic light in the image is red, green, yellow, or unknown.
        # You will need to write the classification code yourself, of
        # course!
        self.light_classifier = None

        # self.algorithm possible values:
        # 0: use light-state info from simulator
        # 1: use light-state info from Calvenn's algorithm
        # If you add a new image-processing algorithm, give it
        # a new number!

        # By default (if no parameter set) use Calvenn's algorithm
        self.algorithm = rospy.get_param("/traffic_light_algorithm", 1)

        if self.algorithm == 0:
            rospy.logwarn("Using traffic-light state from simulator!")

        # Don't process image data if the traffic light is more
        # than max_tl_distance (meters) away.  max_tl_distance <= 0 means
        # process all images
        self.max_tl_distance = -1

        # Call Calvenn's code
        if self.algorithm == 1:
            # import Calvenn's traffic light classifier
            from light_classification_ct.tl_classifier import TLClassifierCT

            self.light_classifier = TLClassifierCT()
            # Which frames to process.  For instance, 
            # skip_factor = 5 means process every 5th frame
            self.light_classifier.skip_factor = 1
            # Don't process image if next light is more than
            # 200 meters away
            self.max_tl_distance = 200
        elif self.algorithm == 2:
            from light_classification_csr.tl_classifier import TLClassifierCSR

            self.light_classifier = TLClassifierCSR()
            # Which frames to process.  For instance,
            # skip_factor = 5 means process every 5th frame
            self.light_classifier.skip_factor = 1
            # Don't process image if next light is more than
            # 200 meters away
            self.max_tl_distance = 200

        # If you add a new image processing algorithm, set
        # self.light_classifier here.

        # A ROS utility that handles coordinate-system transforms.
        # This will be used below to create a transformation from
        # the car camera's coordinate system to the world coordinate
        # system.
        self.listener = tf.TransformListener()

        # Current and previous traffic-light state (green, red, yellow, unknown)
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN

        # Previously-determined waypoint of the stop-line at
        # the next red light in front of the car.
        self.last_wp = -1

        # Number of frames since the traffic-light state changed
        self.state_count = 0

        # Next waypoint currently ahead of the car
        self.next_car_waypoint = -1

        rospy.spin()

    # When a new pose is published (by the vehicle), save a copy
    def pose_cb(self, msg):
        self.pose = msg

    # When the next waypoint in front of the car is published
    # (by update_waypoints.py), save it.
    def next_waypoint_cb(self, msg):
        # print("next_wp", msg.data)
        self.next_car_waypoint = msg.data

    # When the waypoints are published (there are 10,000 in the
    # simulator), store a copy of them.  They should only be
    # published once, and in any case we will store a copy only
    # once, because they don't change during the simulator run.
    # After the copy is made, iterate over the traffic light 
    # stop-lines stored in self.config.  To each stop-line position
    # (which is a two-element list containing x and y of the stop-line),
    # append the nearest waypoint number (so that each position is
    # now a 3-element list, containing x, y, and waypoint number).
    def waypoints_cb(self, waypoints):
        # Only need to do this once, so skip if we've already done this.
        if self.waypoints and self.waypoints.waypoints and len(self.waypoints.waypoints) > 0:
            return

        # Save a copy of the waypoints
        self.waypoints = waypoints

        # Iterate over the stop-line positions
        positions = self.config['stop_line_positions']
        for pos in positions:
            # Each position is just a two-element list containing
            # the x and y of the stop line
            (x, y) = pos

            # Find the id of the nearest waypoint to this position
            pose = point_to_pose(x, y)
            nearest = self.get_closest_waypoint(pose)
            # print("nearest", nearest)

            # Add the waypoint id to pos, so it now contains 3
            # items: x, y, waypoint id
            pos.append(nearest)


    # This callback handles the list of lights sent by the vehicle
    # via the /vehicle/traffic_lights topic.  This list contains
    # the x,y,z position and yaw (?) of each traffic light.  When we
    # use the simulator, the list also gives the state (red, yellow, green)
    # of each light.  Of course, the state information won't be available
    # when we use Udacity's car, though the x,y,z,yaw information will be.
    # The x,y,z,yaw should not change over time, only the state will
    # change.
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """ Udacity comment:
        Identifies red lights in the incoming camera image and 
        publishes, to topic '/traffic_waypoint', the index of the 
        waypoint closest to the red light's stop line.
        If no red light is detected ahead, publish -1.

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        # This routine is where the work takes place to
        # determine the state of the upcoming light
        light_wp, state = self.process_traffic_lights()

        '''
        Udacity comment:
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` 
        number of times till we start using it. Otherwise the previous 
        stable state is used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1*light_wp
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # Comments in triple quotes are from Udacity, but Udacity did not
    # provide the code.
    def get_closest_waypoint(self, pose):
        """ Udacity comment:
        Identifies the closest path waypoint to the given position
        https://en.wikipedia.org/wiki/Closest_pair_of_points_problem

        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """

        # Note:
        # The referenced Wikipedia article is not relevant
        # to the problem that needs to be solved here!

        # This routine will only be called to find the waypoint
        # closest to each stop-line in self.conf.  Thus, it will 
        # probably be called just a dozen times or so.  The nearest 
        # waypoint to the current car position will be provided by the 
        # topic 'next_waypoint', so we don't need to use the code
        # here to determine that information.

        # Nothing fancy, just loop over all 10,000 waypoints and find
        # the closest one.
        ppt = pose_to_point(pose)
        mindist = 0.
        mini = -1
        wps = self.waypoints.waypoints
        for i in range(len(wps)):
            wp = wps[i]
            wpt = waypoint_to_point(wp)
            dsq = point_dist_sq(wpt, ppt)
            if mini < 0 or dsq < mindist:
                mini = i
                mindist = dsq
        return mini

    # TODO TODO You need to complete this routine yourself.
    # This routine takes an x,y,z location (the location of
    # a traffic signal, for instance) and should return the x,y
    # location of where this 3D point would appear in
    # the car camera's image.
    def project_to_image_plane(self, point_in_world):
        """ Udacity comment:
        Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        # Note: according to comments on slack, the
        # focal-length info provided by config below is
        # accurate for the Udacity car, but inaccurate for
        # the simulator.  Someone suggested that for the
        # simulator, fx = fy = 2000 might be approximately
        # true, but you may need to experiment.

        # NEW NOTE: Udacity for unknown reasons has removed
        # these values from the config file for the simulator.
        # Calling these will cause an error
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

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

        #TODO Use tranform and rotation to calculate 2D position of 
        # traffic light in image

        x = 0
        y = 0

        return (x, y)

    # TODO TODO You need to complete this routine yourself.
    # However, most of the work is done in a separate routine,
    # get_classification, which is defined in the file
    # tl_detector/light_classification/tl_classifier.py.
    #
    # TODO TODO You will have to write the code in tl_classifier.py
    # yourself as well.
    def get_light_state(self, light):
        """Udacity comment:
        Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in 
            styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # TODO Note that you have to write most of project_to_image_plane
        # yourself, only a skeleton is provided.

        # New Note: Udacity for reasons best known to itself
        # has broken the project_to_image_plane function.  Don't call,
        # until you have studied the function.
        # x, y = self.project_to_image_plane(light.pose.pose.position)

        # TODO Use light location to zoom in on traffic light in image

        # Your code should create a new targeted_cv_image, zoomed in on 
        # the traffic signal.  As a placeholder, we just copy cv_image
        # to targeted_cv_image, but you should replace this line.
        targeted_cv_image = cv_image

        # TODO your code to create targeted_cv_image goes here

        # The code below publishes targeted_cv_image, so that you can
        # view it and see whether your zoom-in code is working
        # correctly.  To view the targeted image, run the following
        # command:
        #
        # rqt_image_view /targeted_image
        #
        # Comment out the next 3 lines when you no longer need to view 
        # this image.
        # targeted_image_msg = self.bridge.cv2_to_imgmsg(
        #         targeted_cv_image, "bgr8")
        # self.targeted_image_pub.publish(targeted_image_msg)

        # Get classification, using the targeted image
        return self.light_classifier.get_classification(targeted_cv_image)

    # Not much for you TODO here, except comment out the "cheat"
    # line once your traffic-light classifier code is working
    def process_traffic_lights(self):
        """ Udacity comment:
        Finds closest visible traffic light, if one exists, and 
        determines its location and color

        Returns:
            int: index of waypoint closes to the upcoming stop 
                 line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in 
                 styx_msgs/TrafficLight)

        """

        light = None
        light_wp = -1
        state = TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop 
        # in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        car_wp = self.next_car_waypoint
        if car_wp < 0:
            return light_wp, state

        # Find the closest visible traffic light (if one exists)
        positions = self.config['stop_line_positions']
        if len(positions) == 0 or len(positions[0]) < 3 or len(self.lights) == 0:
            return light_wp, state
        for i in range(len(positions)):
            (x, y, wp) = positions[i]
            if wp > car_wp:
                light = self.lights[i]
                light_wp = wp
                break

        # This happens when the car is past the last signal on the
        # track; in this case, look past the end to the first signal:
        if light_wp < 0:
            light_wp = positions[0][2]
            light = self.lights[0]

        if light:


            # If using simulator light-state info:
            if self.algorithm == 0:
                state = light.state
            else:
                # using an algorithm

                # see if traffic light is close enough to
                # be detected; if not, return "unknown" without
                # calling the image processing algorithm
                if self.max_tl_distance > 0:
                    light_pt = pose_to_point(light.pose.pose)
                    car_pt = pose_to_point(self.pose.pose)
                    dist = point_dist(light_pt, car_pt)
                    if dist > self.max_tl_distance:
                        return light_wp, state

                # call the image-processing algorithm
                state = self.get_light_state(light)

            return light_wp, state
            

        return light_wp, state
        

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
