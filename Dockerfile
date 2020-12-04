# Udacity capstone project dockerfile
FROM ros:noetic-robot
LABEL maintainer="olala7846@gmail.com"

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# setup rosdep
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
RUN apt-get install -y gfortran libopenblas-dev liblapack-dev
RUN apt-get install -y python3-pip

# install required ros dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y netbase

COPY requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt


RUN mkdir /capstone3
VOLUME ["/capstone3"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone3/ros
