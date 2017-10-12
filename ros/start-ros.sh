#!/bin/bash

# exit on first error
set -e

catkin_make
source devel/setup.bash
roslaunch launch/styx.launch