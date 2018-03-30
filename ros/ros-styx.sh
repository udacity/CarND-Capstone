#! /bin/bash -e

# Launch ROS in styx mode
# (adapted from Udacity SDC-ND Capstone Project Starter Code)
cd /capstone/ros
source devel/setup.sh
catkin_make
roslaunch launch/styx.launch
