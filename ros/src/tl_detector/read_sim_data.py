#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import waypoint_lib.helper as helper
import os.path
import message_filters
import pickle
import time

if __name__ == '__main__':
    data_pack_folder = os.path.join('.', 'output_images')
    data_pack_name = '20170920182138'

    data_file_name = os.path.join(data_pack_folder, data_pack_name, '{}.pickle'.format(data_pack_name))
    print 'Reading data from: {}'.format(data_file_name)

    rf = open(data_file_name, 'rb')
    data = pickle.load(rf)
    rf.close()

    print "Read len: {}".format(len(data))
    # print 'data[0] = {}'.format(data[0])


    # Example of access
    for rec in data:

        img_filename = rec['filename']
        print 'img_filename = ', img_filename

        pose_msg = rec['pose_msg']
        # print 'pose_msg = ', pose_msg

        traffic_light = rec['traffic_light']
        # print 'traffic_light = ', traffic_light

        light_dist_wp = rec['light_dist_wp']
        print 'light_dist_wp = ', light_dist_wp

        state = rec['state']
        print 'state = ', state

        # Example helper methods access
        yaw = helper.yaw_from_orientation(pose_msg.pose.orientation)
        print 'pose yaw = ', yaw

        break
