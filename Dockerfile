FROM osrf/ros:kinetic-desktop-full

RUN apt-get update

RUN apt-get install -y \
	wget \
	openssh-server \
	vim \
	screen \
	sudo

# Fix missing TCP protocol in python socket
RUN apt-get -y -o Dpkg::Options::="--force-confmiss" install --reinstall netbase

# install pip
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python get-pip.py

# carnd requirements
RUN wget https://raw.githubusercontent.com/eva-carnd/CarND-Capstone/master/requirements.txt
RUN pip install -r requirements.txt

# Dataspeed DBW
RUN wget https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash
RUN chmod +x sdk_install.bash
RUN /bin/bash -c "./ros_entrypoint.sh && echo $ROS_DISTRO && ./sdk_install.bash"
# TODO here the upgrade is not executed with -y option!

#RUN apt-get -y install locales

# Environment / Locales
#ENV LANGUAGE en_US.UTF-8
#ENV LANG en_US.UTF-8
#ENV LC_ALL en_US.UTF-8
#RUN locale-gen en_US.UTF-8
#RUN dpkg-reconfigure locales

# Allow root access over ssh
RUN echo "root:Docker!" | chpasswd
RUN sed -i 's/prohibit-password/yes/' /etc/ssh/sshd_config

# Run startup script to init the workspace
COPY startup.sh ./startup.sh
CMD /bin/bash -c "./startup.sh" && /bin/bash
