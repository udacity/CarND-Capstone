# Udacity capstone project dockerfile
FROM ros:kinetic-robot
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
RUN apt-get install -y python-pip protobuf-compiler python-pil python-lxml python-tk
RUN pip install --no-cache-dir Cython

COPY requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# install required ros dependencies
RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

# other tools
RUN apt-get install -y vim less mc screen

# see: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md
# Tensorflow Object Detection API installation
RUN git clone https://github.com/tensorflow/models.git tensorflow_models
ENV TENSORFLOW_MODELS ${PWD}/tensorflow_models

## COCO API installation
RUN git clone https://github.com/cocodataset/cocoapi.git cocodataset_cocoapi
ENV COCODATASET_COCOAPI ${PWD}/cocodataset_cocoapi
RUN cd ${COCODATASET_COCOAPI}/PythonAPI && \
        make && \
        cp -r pycocotools ${TENSORFLOW_MODELS}/research/

RUN cd ${TENSORFLOW_MODELS}/research && \
        protoc object_detection/protos/*.proto --python_out=.

# Add Libraries to PYTHONPATH
ENV PYTHONPATH ${PYTHONPATH}${TENSORFLOW_MODELS}/research:${TENSORFLOW_MODELS}/research/slim

RUN apt-get autoclean
RUN apt-get clean
RUN apt-get autoremove

RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros
