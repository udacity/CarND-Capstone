#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane, Waypoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf as tf2
import cv2
import pygame
import sys
import numpy as np
import math
import csv
import tensorflow as tf
import yaml
import os

label = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']

class autoTLDataCollector():
    def __init__(self, incsv, outcsv, config_file):
        # initialize and subscribe to the camera image and traffic lights topic
        rospy.init_node('auto_traffic_light_data_collector')

        self.cv_image = None

        self.sub_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # test different raw image update rates:
        # - 2   - 2 frames a second
        # - 0.5 - 1 frame every two seconds
        self.updateRate = 10

        self.waypoints = None
        self.nwp = None
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.i = 0

        self.img_rows = 600
        self.img_cols = 800
        self.img_ch = 3
        self.screen = None
        self.position = None
        self.theta = None
        self.ctl = None
        self.traffic_light_to_waypoint_map = []

        # get traffic light positions
        with open(os.getcwd()+'/src/tl_detector/'+config_file, 'r') as myconfig:
            config_string=myconfig.read()
            self.config = yaml.load(config_string)

        # initialize tensorflow
        self.tf_session = None
        self.predict = None
        self.last_pose = None

        self.poses = []
        self.imagefiles = []
        self.labels = []

        # read in incsv file
        with open(incsv) as f:
            count = 0
            for line in f:
                if count > 0:
                    data = line.split(',')
                    p = PoseStamped()
                    p.pose.position.x = float(data[0])
                    p.pose.position.y = float(data[1])
                    p.pose.position.z = float(data[2])
                    p.pose.orientation.x = float(data[3])
                    p.pose.orientation.y = float(data[4])
                    p.pose.orientation.z = float(data[5])
                    p.pose.orientation.w = float(data[6])
                    self.poses.append(p)
                    self.imagefiles.append(data[7].replace("'", ""))
                    self.labels.append(int(data[8]))
                count += 1

        # set up csv file for labeling
        fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw', 'image', 'label']
        self.log_file = open(outcsv, 'w')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=fieldname)
        self.log_writer.writeheader()

        self.loop()

    def scale(self, x, feature_range=(-1, 1)):
        """Rescale the image pixel values from -1 to 1

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            image (cv::Mat): image rescaled from -1 to 1 pixel values

        """
        # scale to (-1, 1)
        x = ((x - x.min())/(255 - x.min()))

        # scale to feature_range
        min, max = feature_range
        x = x * (max - min) + min
        return x

    def waypoints_cb(self, msg):
        # DONE: Implement
        if self.waypoints is None:
            self.waypoints = []
            for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)

            # just need to get it once
            self.sub_waypoints.unregister()
            self.sub_waypoints = None

            # initialize lights to waypoint map
            self.initializeLightToWaypointMap()

    def initializeLightToWaypointMap(self):
        # find the closest waypoint to the given (x,y) of the triffic light
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        for lidx in range(len(self.config['light_positions'])):
            dist = 100000.
            tlwp = 0
            for widx in range(len(self.waypoints)):
                d1 = dl(self.waypoints[widx].pose.pose.position, self.config['light_positions'][lidx])
                if dist > d1:
                    tlwp = widx
                    dist = d1
            self.traffic_light_to_waypoint_map.append(tlwp)

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
        if self.waypoints is not None:
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

    def update_pygame(self):
        ### initialize pygame
        if self.screen is None:
            pygame.init()
            pygame.display.set_caption("Udacity SDC System Integration Project: Bad Auto Data Collector")
            self.screen = pygame.display.set_mode((self.img_cols,self.img_rows), pygame.DOUBLEBUF)
        ## give us a machine view of the world
        self.sim_img = pygame.image.fromstring(self.cv_image.tobytes(), (self.img_cols, self.img_rows), 'RGB')
        self.screen.blit(self.sim_img, (0,0))
        pygame.display.flip()

    def loop(self):
        # only check once a updateRate time in milliseconds...
        font = cv2.FONT_HERSHEY_COMPLEX
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown() and self.i < len(self.poses):
            self.pose = self.poses[self.i].pose
            self.position = self.pose.position
            euler = tf2.transformations.euler_from_quaternion([
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w])
            self.theta = euler[2]
            tl_dist = self.dist_to_next_traffic_light()

            # set up tensorflow and traffic light classifier
            if self.tf_session is None:
                # get the traffic light classifier
                self.model_path = '../classifier/GAN-Semi-Supervised-site'
                self.tf_config = tf.ConfigProto(log_device_placement=True)
                self.tf_config.gpu_options.per_process_gpu_memory_fraction = 0.2  # don't hog all the VRAM!
                self.tf_config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
                self.tf_session = tf.Session(config=self.tf_config)
                self.saver = tf.train.import_meta_graph(self.model_path + '/checkpoints/generator.ckpt.meta')
                self.saver.restore(self.tf_session, tf.train.latest_checkpoint(self.model_path + '/checkpoints/'))

                # get the tensors we need for doing the predictions by name
                self.tf_graph = tf.get_default_graph()
                self.input_real = self.tf_graph.get_tensor_by_name("input_real:0")
                self.drop_rate = self.tf_graph.get_tensor_by_name("drop_rate:0")
                self.predict = self.tf_graph.get_tensor_by_name("predict:0")

            self.cv_image = cv2.cvtColor(cv2.imread(os.getcwd()+'/../'+self.imagefiles[self.i]), cv2.COLOR_BGR2RGB)
            height = self.cv_image.shape[0]
            width = self.cv_image.shape[1]

            if height != 600 or width != 800:
                image = cv2.resize(self.cv_image, (800, 600), interpolation=cv2.INTER_AREA)
            else:
                image = np.copy(self.cv_image)

            predict = [ TrafficLight.RED ]
            if self.predict is not None:
                predict = self.tf_session.run(self.predict, feed_dict = {
                    self.input_real: self.scale(image.reshape(-1, 600, 800, 3)),
                    self.drop_rate:0.})

            # only save the samples that are not predicted correctly...
            if self.ctl is None:
                self.ctl = None
                print "self.ctl is None! self.i =", self.i
            if tl_dist is None or tl_dist > 80.:
                self.ctl = None
                print "tl_dist is None or > 80! self.i =", self.i
                
            if self.pose is not None and self.ctl is not None:
                if predict[0] != self.labels[self.i]:
                    print "INCORRECT prediction for ", self.i, "label: ", label[self.labels[self.i]], " predicted: ", label[predict[0]]
                    if self.last_pose is None or self.last_pose != self.pose:
                        self.last_pose = self.pose
                        self.log_writer.writerow({
                            'x': self.pose.position.x,
                            'y': self.pose.position.y,
                            'z': self.pose.position.z,
                            'ax': self.pose.orientation.x,
                            'ay': self.pose.orientation.y,
                            'az': self.pose.orientation.z,
                            'aw': self.pose.orientation.w,
                            'image': "'"+self.imagefiles[self.i]+"'",
                            'label': self.labels[self.i]})
                        self.log_file.flush()
                else:
                    print "CORRECT prediction for ", self.i, "label: ", label[self.labels[self.i]], " predicted: ", label[predict[0]]

            if self.ctl is not None:
                color = (255, 255, 0)
                font = cv2.FONT_HERSHEY_COMPLEX
                text1 = "Frame: %d   Traffic Light State: %d"
                text2 = "Nearest Traffic Light (%d)..."
                text3a = "is %fm ahead."
                text3b = "is behind."
                text4 = 'Label: %s    Predicted: %s'
                cv2.putText(self.cv_image, text1%(self.i, self.labels[self.i]), (100, height-240), font, 1, color, 2)
                cv2.putText(self.cv_image, text2%(self.ctl), (100, height-180), font, 1, color, 2)
                if tl_dist is not None:
                    cv2.putText(self.cv_image, text3a%(tl_dist), (100, height-120), font, 1, color, 2)
                else:
                    cv2.putText(self.cv_image, text3b, (100, height-120), font, 1, color, 2)
                cv2.putText(self.cv_image, text4%(label[self.labels[self.i]], label[predict[0]]), (100, height-60), font, 1, color, 2)
                self.update_pygame()

            # schedule next loop
            self.i += 1
            rate.sleep()


if __name__ == "__main__":
    defaultOutput = 'badimages.csv'
    parser = argparse.ArgumentParser(description='Udacity SDC: System Integration - Auto Data Collector with Classifier')
    parser.add_argument('--infilename', type=str, default='loop_with_traffic_light.csv', help='csv input file')
    parser.add_argument('--outfilename', type=str, default=defaultOutput, help='jpeg output file pattern')
    parser.add_argument('--trafficconfig', type=str, default='sim_traffic_light_config.yaml', help='traffic light yaml config')
    args = parser.parse_args()
    incsv = os.getcwd()+'/../test_images/'+args.infilename
    outcsv = os.getcwd()+'/../test_images/'+args.outfilename
    config_file = args.trafficconfig

    try:
        autoTLDataCollector(incsv, outcsv, config_file)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start auto traffric light data collector.')
