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

import numpy as np
import os
import sys
import tarfile
import tensorflow
import zipfile

from collections import defaultdict
from io import StringIO

from utils import label_map_util
from utils import visualization_utils as vis_util
# import matplotlib.pyplot as plt

rospy.init_node('tl_detector')

PATH_TO_CKPT = rospy.get_param('~graph', '')
PATH_TO_LABELS = rospy.get_param('~labels', '')
NUM_CLASSES = rospy.get_param('~classes', 3)

print('Configuration:')
print('  PATH_TO_CKPT',PATH_TO_CKPT)
print('  PATH_TO_LABELS',PATH_TO_LABELS)
print('  NUM_CLASSES',NUM_CLASSES)

if len(PATH_TO_CKPT)==0 or len(PATH_TO_LABELS)==0:
    print('You need to provide 3 parameters: graph, labels and classes')
    exit(-1)

STATE_COUNT_THRESHOLD = 3

detection_graph = tensorflow.Graph()
with detection_graph.as_default():
    od_graph_def = tensorflow.GraphDef()
    with tensorflow.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tensorflow.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

with detection_graph.as_default():
    with tensorflow.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')


        class TLDetector(object):
            def __init__(self):
                self.pose = None
                self.waypoints = None
                self.camera_image = None
                self.lights = []

                self.written_images_counter = 0

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
                sub6 = rospy.Subscriber('/image_color', Image, self.image_cb,queue_size=1,tcp_nodelay=True)

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
                    if state == TrafficLight.RED:
                        light_wp = light_wp
                    else:
                        light_wp = -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                    pass
                self.state_count += 1

            def get_closest_waypoint(self, pose):
                """Identifies the closest path waypoint to the given position
                    https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
                Args:
                    pose (Pose): position to match a waypoint to

                Returns:
                    int: index of the closest waypoint in self.waypoints

                """
                if self.waypoints is None:
                    return 0
                wps = self.waypoints.waypoints

                # wps = [str(x.pose.pose) for x in self.waypoints.waypoints]

                dists = []
                xx = []
                yy = []
                min_id = self.last_wp
                if min_id ==-1:
                    for wp in wps:
                        pos = pose.position
                        _pos = wp.pose.pose.position

                        xx.append(_pos.x)
                        yy.append(_pos.y)

                        d = (pos.x - _pos.x) * (pos.x - _pos.x) + (pos.y - _pos.y) * (pos.y - _pos.y) + (pos.z - _pos.z) * (pos.z - _pos.z)
                        dists.append(d)
                    min_id = dists.index(min(dists))
                    # plt.plot(xx, yy,'r')
                    # plt.plot(pose.position.x,pose.position.y,'gx')
                    # plt.ion()
                    # plt.show()
                else:
                    start = max(min_id-10,0)
                    end = min(min_id+10,len(wps)-1)
                    for wp in wps[start:end]:
                        pos = pose.position
                        _pos = wp.pose.pose.position

                        xx.append(pos.x)
                        yy.append(pos.y)

                        d = (pos.x - _pos.x) * (pos.x - _pos.x) + (pos.y - _pos.y) * (pos.y - _pos.y) + (pos.z - _pos.z) * (pos.z - _pos.z)
                        dists.append(d)
                    min_id = max(min_id-10,0) + dists.index(min(dists))
                    # plt.plot(pose.position.x,pose.position.y,'gx')
                    # plt.ion()
                    # plt.show()


                return min_id


            def project_to_image_plane(self, point_in_world):
                """Project point from 3D world coordinates to 2D camera image location

                Args:
                    point_in_world (Point): 3D location of a point in the world

                Returns:
                    x (int): x coordinate of target point in image
                    y (int): y coordinate of target point in image

                """

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

                #TODO Use tranform and rotation to calculate 2D position of light in image

                x = 0
                y = 0

                return (x, y)

            def process_traffic_lights(self):
                """Finds closest visible traffic light, if one exists, and determines its
                    location and color

                Returns:
                    int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
                    int: ID of traffic light color (specified in styx_msgs/TrafficLight)

                """
                state = TrafficLight.UNKNOWN
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                    # cv_image = cv2.resize(cv_image,(300,300))
                    image = cv2. cvtColor(cv_image,cv2.COLOR_BGR2RGB)

                    image_np_expanded = np.expand_dims(cv2.resize(image.copy(),(300,300)), axis=0)
                    (boxes, scores, classes, num) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})

                    detected_classes=vis_util.visualize_boxes_and_labels_on_image_array(
                      image,
                      np.squeeze(boxes),
                      np.squeeze(classes).astype(np.int32),
                      np.squeeze(scores),
                      category_index,
                      min_score_thresh=0.14,
                      use_normalized_coordinates=True,
                      line_thickness=8)

                    detected_classes = [x[0] for x in detected_classes]
                    detected_lights = [0,0,0,0]
                    for cl in detected_classes:
                        detected_lights[cl] = detected_lights[cl] +1
                    max_id = detected_lights.index(max(detected_lights))

                    if len(detected_lights):
                        if max_id == 1:
                            state = TrafficLight.RED
                        if max_id == 2:
                            state = TrafficLight.YELLOW
                        if max_id == 3:
                            state = TrafficLight.GREEN

                    # print(detected_classes,detected_lights, max_id, 'state:',state)
                    result = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
                    # cv2.imwrite('/tmp/image%08d.jpg'%self.written_images_counter,result)
                    self.written_images_counter = self.written_images_counter + 1
                    # cv2.imshow('result',result)
                    # cv2.waitKey(1)

                except IOError as (errno, strerror):
                    print "I/O error({0}): {1}".format(errno, strerror)
                except ValueError:
                    print "ValueError"
                except:
                    print "Unexpected error:", sys.exc_info()[0]


                # List of positions that correspond to the line to stop in front of for a given intersection
                stop_line_positions = self.config['stop_line_positions']
                car_position = 0

                if state != TrafficLight.UNKNOWN:
                    if(self.pose):
                        car_position = self.get_closest_waypoint(self.pose.pose)

                    #TODO find the closest visible traffic light (if one exists)
                    light_wp = car_position + 1
                    return light_wp, state
                # self.waypoints = None
                return -1, TrafficLight.UNKNOWN

        if __name__ == '__main__':
            try:
                TLDetector()
            except rospy.ROSInterruptException:
                rospy.logerr('Could not start traffic node.')
