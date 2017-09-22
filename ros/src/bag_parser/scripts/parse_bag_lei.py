#!/usr/bin/env python

import csv
import cv2
import os
import math
import numpy as np
import rospy
import rosbag
import tf
import yaml

from cv_bridge import CvBridge


def find_closest_light(car_pose, traffic_lights):
  closest_traffic_light_index = -1
  closest_traffic_light_state = -1
  distance_to_closest_traffic_light = -1

  for traffic_light_index in range(len(traffic_lights)):
    traffic_light = traffic_lights[traffic_light_index]
    traffic_light_position = np.array([traffic_light.pose.pose.position.x,
                                       traffic_light.pose.pose.position.y,
                                       traffic_light.pose.pose.position.z,
                                       1])
    traffic_light_in_car_pose = \
      np.linalg.inv(car_pose).dot(traffic_light_position)

    if traffic_light_in_car_pose[0] > 0:
      distance_to_traffic_light = math.sqrt(
        traffic_light_in_car_pose[0] * traffic_light_in_car_pose[0] +
        traffic_light_in_car_pose[1] * traffic_light_in_car_pose[1])
      if distance_to_closest_traffic_light < 0 \
          or distance_to_traffic_light < distance_to_closest_traffic_light:
        distance_to_closest_traffic_light = distance_to_traffic_light
        closest_traffic_light_index = traffic_light_index
        closest_traffic_light_state = traffic_light.state

  return (closest_traffic_light_index,
          closest_traffic_light_state,
          distance_to_closest_traffic_light)


def parse_rosbag():
  # INPUT_ROSBAG_PATH = "/data/carnd/CarND-Capstone/rosbag/test.bag"
  INPUT_ROSBAG_PATH = "/data/carnd/CarND-Capstone/rosbag/20170916_0_lei.bag"
  assert(os.path.exists(INPUT_ROSBAG_PATH))

  TRAFFIC_LIGHT_CONFIG_PATH = \
    "/fig/home/lei/carnd/CarND-Capstone/CarND-Capstone/ros" \
    + "/src/tl_detector/sim_traffic_light_config.yaml"
  assert(os.path.exists(TRAFFIC_LIGHT_CONFIG_PATH))

  output_folder = "/fig/home/lei/carnd/CarND-Capstone/CarND-Capstone/tl_data/20170916_0_lei/"
  assert(os.path.isdir(output_folder))

  DISTANCE_TO_TRAFFIC_LIGHT_LOWER_BOUND = 0
  DISTANCE_TO_TRAFFIC_LIGHT_UPPER_BOUND = 50
  assert (DISTANCE_TO_TRAFFIC_LIGHT_LOWER_BOUND >= 0)
  assert (DISTANCE_TO_TRAFFIC_LIGHT_UPPER_BOUND > \
          DISTANCE_TO_TRAFFIC_LIGHT_LOWER_BOUND)

  image_folder_name = "images"
  image_folder = output_folder + image_folder_name + "/"
  assert(os.path.isdir(image_folder))

  csv_file_name = "traffic_light_data.csv"
  csv_file_path = output_folder + csv_file_name

  CAR_POSE_TOPIC = '/current_pose'
  IMAGE_TOPIC = '/image_color'
  TRAFFIC_LIGHT_TOPIC = '/vehicle/traffic_lights'

  MSG_TIME_SYNCHRONIZATION_THRESHOLD = rospy.Duration.from_sec(0.02)

  TRAFFIC_LIGHT_RED = 0
  TRAFFIC_LIGHT_YELLOW = 1
  TRAFFIC_LIGHT_GREEN = 2

  print("loading rosbag")
  bag = rosbag.Bag(INPUT_ROSBAG_PATH)
  print("finish loading rosbag")

  with open(TRAFFIC_LIGHT_CONFIG_PATH, 'r') as traffic_light_conifg_file:
    traffic_light_config = yaml.load(traffic_light_conifg_file)

  traffic_light_stop_positions_2d = traffic_light_config['light_positions']

  car_pose_msgs = []
  image_msgs = []
  traffic_light_msgs = []

  print("loading msg from rosbag")
  for topic, msg, time in bag.read_messages(
      topics=[CAR_POSE_TOPIC, IMAGE_TOPIC, TRAFFIC_LIGHT_TOPIC]):
    if msg.header.stamp == rospy.Time(0):
      msg.header.stamp = time
      print(topic + " topic has no timestamp associated in message."
            + " use rosbag time instead for message timestamp")
    assert (msg.header.stamp != rospy.Time(0))

    if topic == CAR_POSE_TOPIC:
      car_pose_msgs.append(msg)
    elif topic == IMAGE_TOPIC:
      image_msgs.append(msg)
    elif topic == TRAFFIC_LIGHT_TOPIC:
      assert (len(traffic_light_stop_positions_2d) == len(msg.lights))

      # substitute traffic light position with traffic light stop position
      for traffic_light_index in range(len(traffic_light_stop_positions_2d)):
        traffic_light_stop_position_2d = \
          traffic_light_stop_positions_2d[traffic_light_index]
        msg.lights[traffic_light_index].pose.pose.position.x = \
          traffic_light_stop_position_2d[0]
        msg.lights[traffic_light_index].pose.pose.position.y = \
          traffic_light_stop_position_2d[1]

      traffic_light_msgs.append(msg)
    else:
      assert (0)
  print("loading msg from rosbag")

  print("car_pose_msgs.size:" + str(len(car_pose_msgs)))
  print("image_msgs.size:" + str(len(image_msgs)))
  print("traffic_light_msgs.size:" + str(len(traffic_light_msgs)))
  assert (len(car_pose_msgs) > 0)
  assert (len(image_msgs) > 0)
  assert (len(traffic_light_msgs) > 0)

  with open(csv_file_path, 'wb') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',')

    car_pose_msg_index = 0
    traffic_light_msg_index = 0

    for image_index in range(len(image_msgs)):
      print("processing image[" + str(image_index) + "/" + \
            str(len(image_msgs)) + "]")

      image_msg = image_msgs[image_index]
      image_timestamp = image_msg.header.stamp

      # skip image if there is no car pose msg or traffic light msg arrived yet
      if car_pose_msgs[car_pose_msg_index].header.stamp > image_timestamp \
          or traffic_light_msgs[traffic_light_msg_index].header.stamp \
              > image_timestamp:
        assert (car_pose_msg_index == 0)
        assert (traffic_light_msg_index == 0)

        car_pose_msg_time = car_pose_msgs[car_pose_msg_index].header.stamp
        traffic_light_msg_time = car_pose_msgs[car_pose_msg_index].header.stamp

        car_pose_msg_time_offset = car_pose_msg_time - image_timestamp
        traffic_light_msg_time_offset = traffic_light_msg_time - image_timestamp

        print("skip image[" + str(image_index) + "]")
        print("image_timestamp[" + str(image_timestamp.to_nsec()) + "]" \
              + " car_pose_msg[" + str(car_pose_msg_time.to_nsec()) \
              + "," + str(car_pose_msg_time_offset.to_nsec()) + "]" \
              + " traffic_light_msg[" + str(traffic_light_msg_time.to_nsec()) \
              + "," + str(traffic_light_msg_time_offset.to_nsec()) + "]")

        continue

      # find car pose msg that is before image
      while car_pose_msg_index + 1 < len(car_pose_msgs) and \
              car_pose_msgs[car_pose_msg_index + 1].header.stamp \
              <= image_timestamp:
        car_pose_msg_index = car_pose_msg_index + 1

      # find traffic light msg that is before image
      while traffic_light_msg_index + 1 < len(traffic_light_msgs) and \
              traffic_light_msgs[traffic_light_msg_index + 1].header.stamp \
              <= image_timestamp:
        traffic_light_msg_index = traffic_light_msg_index + 1

      assert (car_pose_msgs[car_pose_msg_index].header.stamp <= image_timestamp)
      assert (traffic_light_msgs[traffic_light_msg_index].header.stamp \
              <= image_timestamp)

      # find msg that is after image
      if car_pose_msg_index + 1 < len(car_pose_msgs) \
          and traffic_light_msg_index + 1 < len(traffic_light_msgs):
        car_pose_msg_index1 = car_pose_msg_index + 1
        traffic_light_msg_index1 = traffic_light_msg_index + 1
        assert (car_pose_msgs[car_pose_msg_index1].header.stamp \
                > image_timestamp)
        assert (traffic_light_msgs[traffic_light_msg_index1].header.stamp \
                > image_timestamp)

        car_pose_msg0 = car_pose_msgs[car_pose_msg_index]
        car_pose_msg1 = car_pose_msgs[car_pose_msg_index1]

        traffic_light_msg0 = traffic_light_msgs[traffic_light_msg_index]
        traffic_light_msg1 = traffic_light_msgs[traffic_light_msg_index1]

        # time synchronization check
        car_pose_msg_dt = \
          car_pose_msg1.header.stamp - car_pose_msg0.header.stamp
        traffic_light_msg_dt = \
          traffic_light_msg1.header.stamp - traffic_light_msg0.header.stamp
        assert (car_pose_msg_dt.to_sec() > 0)
        assert (traffic_light_msg_dt.to_sec() > 0)

        if car_pose_msg_dt <= MSG_TIME_SYNCHRONIZATION_THRESHOLD \
            and traffic_light_msg_dt <= MSG_TIME_SYNCHRONIZATION_THRESHOLD:
          # interpolate car pose
          car_pose_dt0 = float(
            image_timestamp.to_nsec() - car_pose_msg0.header.stamp.to_nsec())
          car_pose_ratio = car_pose_dt0 / float(car_pose_msg_dt.to_nsec())
          assert (car_pose_ratio >= 0)
          assert (car_pose_ratio < 1)

          car_pose_t0 = np.array([car_pose_msg0.pose.position.x,
                                  car_pose_msg0.pose.position.y,
                                  car_pose_msg0.pose.position.z])
          car_pose_q0 = np.array([car_pose_msg0.pose.orientation.x,
                                  car_pose_msg0.pose.orientation.y,
                                  car_pose_msg0.pose.orientation.z,
                                  car_pose_msg0.pose.orientation.w])

          car_pose_t1 = np.array([car_pose_msg1.pose.position.x,
                                  car_pose_msg1.pose.position.y,
                                  car_pose_msg1.pose.position.z])
          car_pose_q1 = np.array([car_pose_msg1.pose.orientation.x,
                                  car_pose_msg1.pose.orientation.y,
                                  car_pose_msg1.pose.orientation.z,
                                  car_pose_msg1.pose.orientation.w])

          car_pose_t = (1 - car_pose_ratio) * car_pose_t0 \
                       + car_pose_ratio * car_pose_t1
          car_pose_q = tf.transformations.quaternion_slerp(
            car_pose_q0, car_pose_q1, car_pose_ratio)

          car_pose = tf.transformations.quaternion_matrix(car_pose_q)
          car_pose[:3, 3] = car_pose_t

          #  traffic light msg consistency check before and after image
          [traffic_light_index0,
           traffic_light_state0,
           distance_to_traffic_light0] = \
            find_closest_light(car_pose, traffic_light_msg0.lights)
          assert (traffic_light_index0 >= 0)
          assert (traffic_light_state0 >= 0)
          assert (distance_to_traffic_light0 >= 0)

          [traffic_light_index1,
           traffic_light_state1,
           distance_to_traffic_light1] = \
            find_closest_light(car_pose, traffic_light_msg1.lights)
          assert (traffic_light_index1 >= 0)
          assert (traffic_light_state1 >= 0)
          assert (distance_to_traffic_light1 >= 0)

          if traffic_light_index0 == traffic_light_index1 and \
                  traffic_light_state0 == traffic_light_state1:
            assert (distance_to_traffic_light0 == distance_to_traffic_light1)

            if distance_to_traffic_light0 >= DISTANCE_TO_TRAFFIC_LIGHT_LOWER_BOUND \
                and distance_to_traffic_light0 <= DISTANCE_TO_TRAFFIC_LIGHT_UPPER_BOUND:
              print("generating training data." \
                    + " image_index:" + str(image_index) \
                    + " car_pose_index:" + str(car_pose_msg_index) \
                    + " traffic_light_index:" + str(traffic_light_msg_index))

              cv2_img = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")

              image_name = "frame_" + str(image_timestamp.to_nsec()) \
                           + "_" + str(traffic_light_index0) \
                           + "_" + str(traffic_light_state0) \
                           + "_" + str(int(distance_to_traffic_light0)) \
                           + ".jpeg"
              image_path = image_folder + image_name

              csv_writer.writerow([image_name,
                                   traffic_light_index0,
                                   traffic_light_state0,
                                   distance_to_traffic_light0])

              success = cv2.imwrite(image_path, cv2_img)
              assert(success)
            else:
              print("distance_to_traffic_light[" \
                    + str(distance_to_traffic_light0) + "]" \
                    + " is not in range [" \
                    + str(DISTANCE_TO_TRAFFIC_LIGHT_LOWER_BOUND) \
                    + "," + str(DISTANCE_TO_TRAFFIC_LIGHT_UPPER_BOUND) + "]")
          else:
            print("closest traffic light consistency check faild. "
                  + (" traffic_light0[" + str(traffic_light_index0) \
                     + "," + str(traffic_light_state0) \
                     + "," + str(distance_to_traffic_light0)) \
                  + (" traffic_light1[" + str(traffic_light_index1) \
                     + "," + str(traffic_light_state1) \
                     + "," + str(distance_to_traffic_light1)))
        else:
          print("msg time synchronization failed." \
                + " thresh:" + str(MSG_TIME_SYNCHRONIZATION_THRESHOLD.to_sec()) \
                + " car_pose_msg_dt:" + str(car_pose_msg_dt.to_sec()) \
                + " traffic_light_msg_dt:" + str(traffic_light_msg_dt.to_sec()))
      else:
        print("reach end of msgs." \
              + (" car_pose_msg[" + str(car_pose_msg_index) \
                 + "/" + str(len(car_pose_msgs)) + "]") \
              + (" traffic_light_msg[" + str(traffic_light_msg_index) \
                 + "/" + str(len(traffic_light_msgs)) + "]"))

    csv_file.close()
  bag.close()


if __name__ == "__main__":
  parse_rosbag()
