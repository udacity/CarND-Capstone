#! /bin/bash -e

# Parameters (optional):
#  $1 = Name of bag to replay (bags are expected to be in "./tl-training/")
bag="traffic_light_training"
if [ ! -z "$1" ]; then bag=$1; fi

# Replay ROS bag
# (adapted from Udacity SDC-ND Capstone Project Starter Code)
echo "Replaying (tl-training/$bag)..."
cd /capstone
source ros/devel/setup.sh
rosbag play -l tl-training/"$bag".bag
