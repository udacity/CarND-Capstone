#!/usr/bin/env python

import argparse
import cv2
import rospy
import math
import numpy as np
import os
import tf
import yaml

from custom_logger import logger
from renderer import PyGameScreen

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image


class DiagnosticsScreen:
    def __init__(self):
        rospy.init_node('diag_gps')
        self.screen = PyGameScreen(width=800, height=800, caption='SDC Fanatics: Diagnostic Screen')
        self.updateRate = 50

        self.config = None
        self.bridge = CvBridge()

        self.load_traffic_light_pos()

        # Set the subscribers for getting the data
        self.waypoints = None
        self.waypoints_initialized = False
        self.sub_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.traffic_lights_initialized = False
        self.sub_traffic_lights = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        self.current_position_initialized = False
        self.sub_current_position = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.sub_raw_camera = rospy.Subscriber('/image_color', Image, self.image_cb)

        # Params
        self.position = None
        self.theta = None

        self.lights = []
        self.traffic_light_to_waypoint_map = []

        self.camera_image = None

        # Run the Loop finally after everything has been initialized
        self.loop()

    def all_initialized(self):
        all_cbs = [self.waypoints_initialized,
                   self.traffic_lights_initialized,
                   self.current_position_initialized]

        return all(all_cbs)

    def load_traffic_light_pos(self, config_file='sim_traffic_light_config.yaml'):
        # get waypoint configuration
        with open(os.getcwd() + '/src/tl_detector/' + config_file, 'r') as myconfig:
            config_string = myconfig.read()
            self.config = yaml.load(config_string)

    def pose_cb(self, msg):
        self.pose = msg.pose
        self.position = self.pose.position
        euler = tf.transformations.euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w])
        self.theta = euler[2]
        self.current_position_initialized = True

    def traffic_cb(self, msg):
        self.lights = msg.lights
        self.traffic_lights_initialized = True

    def waypoints_cb(self, msg):
        logger.info('WayPoints Callback called')
        if self.waypoints is None:
            self.waypoints = []
            for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)

            # create the polyline that defines the track
            x = []
            y = []
            for i in range(len(self.waypoints)):
                x.append(self.waypoints[i].pose.pose.position.x)
                y.append(self.screen.height - (self.waypoints[i].pose.pose.position.y - self.screen.canvas.height))

            self.XYPolyline = np.column_stack((x, y)).astype(np.int32)

            # just need to get it once
            self.sub_waypoints.unregister()
            self.sub_waypoints = None

            # initialize lights to waypoint map
            self.initialize_light_to_waypoint_map()
            self.waypoints_initialized = True

    def image_cb(self, msg):
        """Grab the first incoming camera image and saves it

        Args:
            msg (Image): image from car-mounted camera

        """
        # unregister the subscriber to throttle the images coming in
        # if self.sub_raw_camera is not None:
        #     self.sub_raw_camera.unregister()
        #     self.sub_raw_camera = None

        # fixing convoluted camera encoding...
        if hasattr(msg, 'encoding'):
            if msg.encoding == '8UC3':
                msg.encoding = "rgb8"
        else:
            msg.encoding = 'rgb8'

        self.camera_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def initialize_light_to_waypoint_map(self):
        # find the closest waypoint to the given (x,y) of the triffic light
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        for lidx in range(len(self.config['stop_line_positions'])):
            dist = 100000.
            tlwp = 0
            for widx in range(len(self.waypoints)):
                d1 = dl(self.waypoints[widx].pose.pose.position, self.config['stop_line_positions'][lidx])
                if dist > d1:
                    tlwp = widx
                    dist = d1
            self.traffic_light_to_waypoint_map.append(tlwp)

    def draw_waypoints(self, image, size=5, size2=10):
        color = (128, 128, 128)
        color2 = (255, 0, 0)
        cv2.polylines(image, [self.XYPolyline], 0, color, size)
        lastwp = len(self.waypoints)-1
        x = int(self.waypoints[lastwp].pose.pose.position.x)
        y = int(self.screen.height - (self.waypoints[lastwp].pose.pose.position.y - self.screen.canvas.height))
        cv2.circle(image, (x, y), size2,  color2, -1)

    def drawTrafficLights(self, image, size=10):
        font = cv2.FONT_HERSHEY_COMPLEX
        for i in range(len(self.lights)):
            x = self.lights[i].pose.pose.position.x
            y = self.lights[i].pose.pose.position.y
            if self.lights[i].state == 0:
                color = (255, 0, 0)
            elif self.lights[i].state == 1:
                color = (255, 255, 0)
            else:
                color = (0, 255, 0)
            cv2.circle(image, (int(x), int(self.screen.height-(y-self.screen.canvas.height))), size, color, -1)
            cv2.putText(image, "%d" % i, (int(x-10), int(self.screen.height-(y-self.screen.canvas.height)+40)), font, 1,
                        color, 2)

    def drawCurrentPos(self, image, size=10):
        color = (255, 255, 255)
        cv2.circle(image, (int(self.position.x),
                         int(self.screen.height-(self.position.y-self.screen.canvas.height))), size, color, -1)

    def drawCameraImage(self, image):
        if self.camera_image is not None:
            logger.info('Rendered a Camera Image')
            camera_img_width = 800
            camera_img_height = 600
            offset_y = 700
            offset_x = 450
            image[self.screen.height // 3 + offset_y:self.screen.height // 3 + offset_y + camera_img_height,
                  self.screen.width // 2 + offset_x :self.screen.width // 2 + offset_x + camera_img_width] = \
                cv2.resize(self.camera_image, (camera_img_width, camera_img_height), interpolation=cv2.INTER_AREA)
            # self.camera_image = None

    def draw(self, image):
        if not self.all_initialized():
            return

        self.draw_waypoints(image)
        self.drawTrafficLights(image)
        self.drawCurrentPos(image)
        self.drawCameraImage(image)

    def loop(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            self.screen.clear_canvas()
            if not self.all_initialized():
                logger.critical('Initializing... Start Simulator')
                self.screen.write_text("Initializing.... Start Simulator", self.screen.get_center_of_screen())
                self.screen.update()
                continue
            self.draw(self.screen.get_canvas().get())
            self.screen.update()

        rate.sleep()


if __name__ == "__main__":
    try:
        DiagnosticsScreen()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not diagnostics.')