# Udacity capstone project dockerfile
FROM ros:kinetic-robot

# Install system dependencies
RUN apt update && apt install -y netbase zsh wget curl git vim tmux python-yaml
RUN wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh && sh install.sh
RUN chsh -s `which zsh`
RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && ~/.fzf/install

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# Setup rosdep
RUN sh -c \
    'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
RUN apt-get upgrade -y

# Install python packages
RUN apt-get install -y python-pip
RUN pip install --upgrade "pip < 21.0"
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

# Install required ROS dependencies
RUN apt-get install -y              \
    ros-$ROS_DISTRO-cv-bridge       \
    ros-$ROS_DISTRO-pcl-ros         \
    ros-$ROS_DISTRO-image-proc

RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros
