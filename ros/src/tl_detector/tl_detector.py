#!/usr/bin/env python

from Queue import Queue
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose
from light_classification.tl_classifier import TLClassifier
from route_traffic_lights import RouteTrafficLights, RouteTrafficLight
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from styx_msgs.msg import TrafficLightArray, TrafficLight
import rospy
import yaml

# Use state from traffic lights topic.
USE_GROUND_TRUTH = False
# Min distance (in waypoints) when to start detecting traffic light.
LIGHT_DETECTION_DISTANCE = 200
# Number of detection before a state change is accepted.
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    ''' Contructor.
    '''
    def __init__(self):
        rospy.init_node('tl_detector')

        self.use_ground_truth = USE_GROUND_TRUTH

        # Traffic light classifier's thread and its task queue.
        self.image_processing = False
        self.queue = Queue()
        self.light_classifier = TLClassifier(self.queue, self.classified_cb)
        self.light_classifier.daemon = True
        self.light_classifier.start()

        self.config = yaml.load(rospy.get_param("/traffic_light_config"))
        self.cv_bridge = CvBridge()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.next_waypoint_ahead = None
        self.waypoints = None
        self.traffic_lights = RouteTrafficLights()

        # Subscribers.
        sub0 = rospy.Subscriber('/next_waypoint_ahead', Int32, self.wp_cb)
        sub1 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub2 = rospy.Subscriber('/image_color', Image, self.image_cb)
        '''
        /vehicle/traffic_lights provides the location of the traffic light in 3D
        map space and helps to acquire an accurate ground truth data source for
        the traffic light classifier by sending the current color state of all
        traffic lights in the simulator. When testing on the vehicle, the color
        state will not be available.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.traffic_cb)

        # Publishers.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint',
                                                      Int32, queue_size=1)
        rospy.spin()

    def wp_cb(self, msg):
        ''' Dispatches a message from topic /next_waypoint_ahead.
        Args:
            msg (Int32): Index of the next waypoint ahead.
        '''
        self.next_waypoint_ahead = msg.data

    def waypoints_cb(self, waypoints):
        ''' Dispatches a message from topic /base_waypoints.
        Args:
            waypoints (Lane): Waypoints of the track.
        '''
        self.waypoints = list(waypoints.waypoints)
        # Reset traffic light information on each route update.
        self.traffic_lights.reset(self.waypoints,
                                  self.config['stop_line_positions'])

    def traffic_cb(self, msg):
        ''' Dispatches a message from topic /vehicle/traffic_lights.
        Args:
            msg (TrafficLightArray): Traffic light data.
        '''
        self.traffic_lights.update_states(msg.lights)

    def image_cb(self, msg):
        ''' Dispatches a message from topic /image_color.
        Args:
            msg (Image): image from car-mounted camera.
        '''
        light_wp = -1
        light_state = TrafficLight.UNKNOWN
        light_key = None
        if self.next_waypoint_ahead:
            # Get next traffic light ahead.
            light_wp, light_key = \
                self.traffic_lights.get_next_en_route(self.next_waypoint_ahead)

        if light_key == None:
            # Next light key is unknown so as the light state.
            self.publish_red_light(-1, light_state)
        else:
            stop_wp = self.traffic_lights[light_key].stop_point
            # Distance in waypoints in a circular track.
            d = lambda x,y,s: (x - y + s) % s
            if d(light_wp, self.next_waypoint_ahead, len(self.waypoints)) \
                    > LIGHT_DETECTION_DISTANCE:
                # Next traffic light is too far, its state is unknown.
                self.publish_red_light(stop_wp, light_state)
            else:
                # Get state of next waypoint from either simulator or image.
                if self.use_ground_truth:
                    self.detect_light_state_from_gt(stop_wp, light_key)
                else:
                    self.detect_light_state_from_img(stop_wp, msg)

    def classified_cb(self, stop_wp, light_state):
        ''' Dispatches the traffic light classification result.
        Args:
            stop_wp (Int32): Index of the traffic light's stop waypoint.
            light_state (TrafficLight): State of the traffic light.
        '''
        self.image_processing = False
        self.publish_red_light(stop_wp, light_state)

    def detect_light_state_from_img(self, stop_wp, image_msg):
        ''' Detects and classifies traffic lights in the image message.
        Args:
            stop_wp (Int32): Index of the traffic light's stop waypoint.
            image_msg (Image): Image from car-mounted camera.
        '''
        if not self.image_processing:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.image_processing = True
            self.queue.put((stop_wp, image))

    def detect_light_state_from_gt(self, stop_wp, light_key):
        ''' Detects and classifies traffic lights from the ground truth.
        Args:
            stop_wp (Int32): Index of the traffic light's stop waypoint.
            light_key: Index of the traffic light.
        '''
        light_state = self.traffic_lights[light_key].state
        self.publish_red_light(stop_wp, light_state)

    def publish_red_light(self, stop_wp, light_state):
        ''' Publishes the index of the waypoint closest to the red light's stop
            line to /traffic_waypoint. Each predicted state has to occur
            STATE_COUNT_THRESHOLD number of times till we start using it.
            Otherwise the previous stable state is used.
        Args:
            stop_wp (Int32): Index of the traffic light's stop waypoint.
            light_state (TrafficLight): State of the traffic light.
        '''
        rospy.logdebug("publish_red_light: stop_wp %d, light_state %d",
                       stop_wp, light_state)
        if self.state != light_state:
            self.state_count = 0
            self.state = light_state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_wp = stop_wp \
                if light_state in (TrafficLight.RED, TrafficLight.YELLOW) \
                else -1
            self.last_wp = stop_wp
            self.upcoming_red_light_pub.publish(Int32(stop_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start traffic node.")
