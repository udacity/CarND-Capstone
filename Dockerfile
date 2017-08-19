FROM osrf/ros:indigo-desktop-full

RUN apt-get update

RUN apt-get install -y \
	wget

# install pip
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python get-pip.py

# carnd requirements
RUN git clone https://github.com/eva-carnd/CarND-Capstone.git
RUN pip install -r CarND-Capstone/requirements.txt

RUN ./ros_entrypoint.sh

# Dataspeed DBW
RUN wget https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash
RUN chmod +x sdk_install.bash
RUN /bin/bash -c "./ros_entrypoint.sh && echo $ROS_DISTRO && ./sdk_install.bash"
# TODO here the upgrade is not executed with -y option!

# Environment / Locales
ENV LANGUAGE en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales
