#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import cv2
import pygame
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import yaml
import os

MPS = 0.44704

class GenerateDiagnostics():
    def __init__(self, img_vis_ratio, max_history, text_spacing, font_size, rate, camera_topic, config_file):
        # initialize and subscribe to the camera image and traffic lights topic
        rospy.init_node('diag_gps')

        self.restricted_speed = 10.
        self.cv_image = None
        self.camera_image = None
        self.lights = []
        self.i = 0
        self.nwp = None
        self.ctl = 0

        # lists for storing history values
        self.frame_history = []
        self.vel_history = []
        self.steering_history = []
        self.throttle_history = []
        self.brake_history = []
        self.max_history_size = max_history

        # get waypoint configuration
        with open(os.getcwd()+'/src/tl_detector/'+config_file, 'r') as myconfig:
            config_string=myconfig.read()
            self.config = yaml.load(config_string)

        self.sub_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.sub_fwaypoints = rospy.Subscriber('/final_waypoints', Lane, self.fwaypoints_cb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub_current_velocity = rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.steering_pub = rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steering_cb)
        self.throttle_pub = rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cb)
        self.brake_pub = rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)
        self.sub_traffic_lights = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.camera_topic = camera_topic
        self.sub_raw_camera = None
        self.bridge = CvBridge()

        # test different raw image update rates:
        self.updateRate = rate # rate (Hz) every second

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.current_linear_velocity = 0.
        self.current_angular_velocity = 0.

        self.img_rows = 2500
        self.img_cols = 2500
        self.img_ch = 3
        self.steering_cmd = 0.
        self.waypoints = None
        self.traffic_light_to_waypoint_map = []
        self.fwaypointsx = []
        self.fwaypointsy = []
        self.fwaypointss = []
        self.fwaypointx = 0.
        self.fwaypointy = 0.
        self.screen = None
        self.position = None
        self.theta = None
        self.lights = []
        self.throttle_cmd = None
        self.steering_cmd = None
        self.brake_cmd = None

        # parameters for adjusting the output window size and text output
        self.img_vis_ratio = img_vis_ratio
        self.img_vis_font_size = font_size
        self.img_vis_txt_x = text_spacing
        self.img_vis_txt_y = text_spacing

        # reset the max_open_warning
        plt.rcParams.update({'figure.max_open_warning': 0})
        self.loop()

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        fx = self.config.camera_info.focal_length_x
        fy = self.config.camera_info.focal_length_y

        image_width = self.config.camera_info.image_width
        image_height = self.config.camera_info.image_height

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
        print "trans: ", trans
        print "rot: ", rot
        wp = np.array([ point_in_world.x, point_in_world.y, point_in_world.z ])
        print "point_in_world: ", (wp + trans)


        x = 0
        y = 0
        return (x, y)

    def draw_light_box(self, light):
        """Draw boxes around traffic lights

        Args:
            light (TrafficLight): light to classify

        Returns:
            image with boxes around traffic lights

        """
        (x,y) = self.project_to_image_plane(light.pose.pose.position)

        # use light location to draw box around traffic light in image
        print "x, y:", x, y

    def image_cb(self, msg):
        """Grab the first incoming camera image and saves it

        Args:
            msg (Image): image from car-mounted camera

        """
        # unregister the subscriber to throttle the images coming in
        if self.sub_raw_camera is not None:
            self.sub_raw_camera.unregister()
            self.sub_raw_camera = None
        if len(self.lights) > 0:
            height = int(msg.height)
            width = int(msg.width)

            # fixing convoluted camera encoding...
            if hasattr(msg, 'encoding'):
                if msg.encoding == '8UC3':
                    msg.encoding = "rgb8"
            else:
                msg.encoding = 'rgb8'

            self.camera_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # print "lights:", self.lights

    def steering_cb(self, msg):
        self.steering_cmd = msg.steering_wheel_angle_cmd
        if len(self.steering_history)>self.max_history_size:
            self.steering_history.pop(0)
        self.steering_history.append(self.steering_cmd*10.)

    def throttle_cb(self, msg):
        self.throttle_cmd = msg.pedal_cmd
        if len(self.throttle_history)>self.max_history_size:
            self.throttle_history.pop(0)
        self.throttle_history.append(self.throttle_cmd)

    def brake_cb(self, msg):
        self.brake_cmd = msg.pedal_cmd
        if len(self.brake_history)>self.max_history_size:
            self.brake_history.pop(0)
        self.brake_history.append(self.brake_cmd)

    def pose_cb(self, msg):
        self.i += 1
        self.pose = msg.pose
        self.position = self.pose.position
        euler = tf.transformations.euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w])
        self.theta = euler[2]

    def velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z
        if len(self.vel_history)>self.max_history_size:
            self.vel_history.pop(0)
            self.frame_history.pop(0)
        self.vel_history.append(self.current_linear_velocity)
        self.frame_history.append(self.i)

    def waypoints_cb(self, msg):
        # DONE: Implement
        if self.waypoints is None:
            self.waypoints = []
            for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)

            # No wrapping for this project!
            # # make sure we wrap!
            # self.waypoints.append(msg.waypoints[0])
            # self.waypoints.append(msg.waypoints[1])

            # create the polyline that defines the track
            x = []
            y = []
            for i in range(len(self.waypoints)):
                x.append(self.waypoints[i].pose.pose.position.x)
                y.append(self.img_rows-(self.waypoints[i].pose.pose.position.y-1000.))
            self.XYPolyline = np.column_stack((x, y)).astype(np.int32)

            # just need to get it once
            self.sub_waypoints.unregister()
            self.sub_waypoints = None

            # initialize lights to waypoint map
            self.initializeLightToWaypointMap()

    def initializeLightToWaypointMap(self):
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

    def fwaypoints_cb(self, msg):
        # DONE: Implement
        waypoints = []
        fx = []
        fy = []
        fs = []
        for i in range(len(msg.waypoints)):
            fx.append(float(msg.waypoints[i].pose.pose.position.x))
            fy.append(self.img_rows-(float(msg.waypoints[i].pose.pose.position.y)-1000.))
            fs.append(int(msg.waypoints[i].twist.twist.linear.x/(self.restricted_speed*MPS)*255))
        self.fwaypointsx = fx
        self.fwaypointsy = fy
        self.fwaypointss = fs
        self.fwaypointx = fx[0]
        self.fwaypointy = fy[0]

    def nextWaypoint(self, pose):
        """Identifies the next path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the next waypoint in self.waypoints

        """
        #DONE implement
        location = pose.position
        dist = 100000.
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        nwp = 0
        for i in range(len(self.waypoints)):
            d1 = dl(location, self.waypoints[i].pose.pose.position)
            if dist > d1:
                nwp = i
                dist = d1
        x = self.waypoints[nwp].pose.pose.position.x
        y = self.waypoints[nwp].pose.pose.position.y
        heading = np.arctan2((y-location.y), (x-location.x))
        angle = np.abs(self.theta-heading)
        if angle > np.pi/4.:
            nwp += 1
            if nwp >= len(self.waypoints):
                nwp = 0
        return nwp

    def getNextLightWaypoint(self):
        # find the closest waypoint from our pre-populated waypoint to light map
        tlwp = None
        self.nwp = self.nextWaypoint(self.pose)
        for ctl in range(len(self.traffic_light_to_waypoint_map)):
            # make sure its forward in our direction
            if self.nwp < self.traffic_light_to_waypoint_map[ctl] and tlwp is None:
                tlwp = self.traffic_light_to_waypoint_map[ctl]
                self.ctl = ctl
        return tlwp

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def dist_to_next_traffic_light(self):
        dist = None
        tlwp = self.getNextLightWaypoint()
        if tlwp is not None:
            dist = self.distance(self.waypoints, self.nwp, tlwp)
        return dist

    def drawWaypoints(self, img, size=5, size2=10):
        color = (128, 128, 128)
        color2 = (128, 0, 0)
        cv2.polylines(img, [self.XYPolyline], 0, color, size)
        lastwp = len(self.waypoints)-1
        x = int(self.waypoints[lastwp].pose.pose.position.x)
        y = int(self.img_rows-(self.waypoints[lastwp].pose.pose.position.y-1000.))
        cv2.circle(img, (x, y), size2,  color2, -1)

    def drawFinalWaypoints(self, img, size=1, size2=15):
        for i in range(len(self.fwaypointsx)):
            if self.fwaypointss[i] > 0:
                color = (0, 192, 0)
            else:
                color = (192, 0, 0)
            cv2.circle(img, (int(self.fwaypointsx[i]), int(self.fwaypointsy[i])), size, color, -1)
        if len(self.fwaypointsx) > 0:
            if self.fwaypointss[i] > 0:
                color = (0, 192, 0)
            else:
                color = (192, 0, 0)
            cv2.circle(img, (int(self.fwaypointsx[0]), int(self.fwaypointsy[0])), size2, color, -1)

    def drawTrafficLights(self, img, size=10):
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
            cv2.circle(img, (int(x), int(self.img_rows-(y-1000))), size, color, -1)
            cv2.putText(img, "%d"%(i), (int(x-10), int(self.img_rows-(y-1000)+40)), font, 1, color, 2)

    def drawCurrentPos(self, img, size=10):
        color = (255, 255, 255)
        cv2.circle(img, (int(self.position.x),
                         int(self.img_rows-(self.position.y-1000))), size, color, -1)

    def loop(self):
        # only check once a updateRate time in milliseconds...
        font = cv2.FONT_HERSHEY_COMPLEX
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            if self.theta is not None:
                tl_dist = self.dist_to_next_traffic_light()
                if self.sub_raw_camera is None and tl_dist is not None:
                    if tl_dist < 80.:
                        self.sub_raw_camera = rospy.Subscriber(self.camera_topic, Image, self.image_cb)

                if (self.sub_waypoints is None and self.steering_cmd is not None and
                        self.throttle_cmd is not None and self.brake_cmd is not None):
                    self.cv_image = np.zeros((self.img_rows, self.img_cols, self.img_ch), dtype=np.uint8)
                    self.drawWaypoints(self.cv_image)
                    self.drawFinalWaypoints(self.cv_image)
                    self.drawTrafficLights(self.cv_image)
                    self.drawCurrentPos(self.cv_image)
                    color = (192, 192, 0)
                    text0 = "Frame: %d"
                    text1a = "Nearest Traffic Light (%d) is %fm ahead."
                    text1b = "Nearest Traffic Light (%d) is behind us."
                    text2 = "Curr. position is (%f, %f, %f)."
                    text3 = "Curr. Vehicle Yaw: %f    Linear Vel.: %f  Angular Vel.: %f"
                    text4 = "Curr. Steering Ang.: %f  Throttle: %f     Brake: %f"
                    text5 = "Next Waypoint position is (%f, %f) with %d array len."
                    cv2.putText(self.cv_image, text0%(self.i), (self.img_vis_txt_x,  self.img_vis_txt_y), font, self.img_vis_font_size, color, 2)
                    if tl_dist is not None:
                        cv2.putText(self.cv_image, text1a%(self.ctl, tl_dist), (self.img_vis_txt_x,  self.img_vis_txt_y*2), font, self.img_vis_font_size, color, 2)
                    else:
                        cv2.putText(self.cv_image, text1b%(self.ctl), (self.img_vis_txt_x,  self.img_vis_txt_y*2), font, self.img_vis_font_size, color, 2)
                    cv2.putText(self.cv_image, text2%(self.position.x, self.position.y, self.position.z),  (self.img_vis_txt_x,  self.img_vis_txt_y*3), font, self.img_vis_font_size, color, 2)
                    cv2.putText(self.cv_image, text3%(self.theta, self.current_linear_velocity, self.current_angular_velocity),  (self.img_vis_txt_x, self.img_vis_txt_y*4), font, self.img_vis_font_size, color, 2)
                    cv2.putText(self.cv_image, text4%(self.steering_cmd, self.throttle_cmd, self.brake_cmd),  (self.img_vis_txt_x, self.img_vis_txt_y*5), font, self.img_vis_font_size, color, 2)
                    cv2.putText(self.cv_image, text5%(self.fwaypointx, self.fwaypointy, len(self.fwaypointsx)),  (self.img_vis_txt_x, self.img_vis_txt_y*6), font, self.img_vis_font_size, color, 2)

                    # Output plots for velocity/steering/throttle/brake but only if we have enough data points...
                    mindata = min([len(self.frame_history), len(self.vel_history), len(self.steering_history), len(self.throttle_history), len(self.brake_history)]) - 5
                    brake_max = max(self.brake_history)
                    right_axis_color = 'g'
                    if brake_max > 10.:
                        right_axis_color = 'r'
                    if mindata > self.max_history_size//2:
                        fig = plt.figure(figsize=(16, 6), dpi=100)
                        ax1 = fig.add_subplot(111)
                        p1 = ax1.plot(self.frame_history[:mindata],self.vel_history[:mindata], color='c', label='Velocity (m/s)')
                        p2 = ax1.plot(self.frame_history[:mindata],self.steering_history[:mindata], color='b', label='Steering')
                        ax1.set_ylabel('Velocity (m/s)\nSteering X 10', color='b')
                        for tl in ax1.get_yticklabels():
                            tl.set_color('b')
                        ax2 = ax1.twinx()
                        p3 = ax2.plot(self.frame_history[:mindata],self.brake_history[:mindata], 'r', label='Brake')
                        p4 = ax2.plot(self.frame_history[:mindata],self.throttle_history[:mindata], color='g', label='Throttle')
                        ax2.set_ylabel('Throttle\nBrake', color=right_axis_color)
                        ps = p1 + p2 + p3 + p4
                        lps = [l.get_label() for l in ps]
                        plt.legend(ps, lps, loc=2)
                        for tl in ax2.get_yticklabels():
                            tl.set_color(right_axis_color)

                        fig.canvas.draw()
                        # Now we can save it to a numpy array.
                        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
                        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                        self.cv_image[self.img_rows//3*2-200:self.img_rows//3*2+400, self.img_cols//2-800:self.img_cols//2+800] = data
                        plt.cla()

                    if self.camera_image is not None:
                        self.cv_image[self.img_rows//3:self.img_rows//3+600, self.img_cols//2-400:self.img_cols//2+400] = cv2.resize(self.camera_image, (800,600), interpolation=cv2.INTER_AREA)
                        self.camera_image = None
                    self.update_pygame()
            # schedule next loop
            rate.sleep()

    def update_pygame(self):
        ### initialize pygame
        if self.screen is None:
            pygame.init()
            pygame.display.set_caption("Udacity SDC System Integration Project: Vehicle Diagnostics")
            self.screen = pygame.display.set_mode((self.img_cols//self.img_vis_ratio,self.img_rows//self.img_vis_ratio), pygame.DOUBLEBUF)
        ## give us a machine view of the world
        self.sim_img = pygame.image.fromstring(cv2.resize(self.cv_image,(self.img_cols//self.img_vis_ratio, self.img_rows//self.img_vis_ratio),
            interpolation=cv2.INTER_AREA).tobytes(), (self.img_cols//self.img_vis_ratio, self.img_rows//self.img_vis_ratio), 'RGB')
        self.screen.blit(self.sim_img, (0,0))
        pygame.display.flip()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Diagnostic Screen')
    parser.add_argument('--screensize', type=int, default="4", help='Screen sizes: 1:2500x2500px, 2:1250x1250px, 3:833x833px, 4:625x625px, 5:500x500px ')
    parser.add_argument('--maxhistory', type=int, default="200", help='Maximum History: default=200')
    parser.add_argument('--textspacing', type=int, default="100", help='Text Spacing: default=100')
    parser.add_argument('--fontsize', type=float, default="2", help='Font Size: default=2')
    parser.add_argument('--cameratopic', type=str, default='/image_color', help='camera ros topic')
    parser.add_argument('--trafficconfig', type=str, default='sim_traffic_light_config.yaml', help='traffic light yaml config')
    parser.add_argument('--rate', type=int, default='1', help='refresh rate in Hz')
    args = parser.parse_args()

    try:
        GenerateDiagnostics(int(args.screensize), int(args.maxhistory), int(args.textspacing), float(args.fontsize), int(args.rate), args.cameratopic, args.trafficconfig)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start front camera viewer.')
