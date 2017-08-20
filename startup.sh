#!/bin/bash

/etc/init.d/ssh start
./ros_entrypoint.sh
cd CarND-Capstone/ros
catkin_make
echo "source /CarND-Capstone/ros/devel/setup.bash" >> /root/.bashrc
