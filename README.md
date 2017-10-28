# Introduction
This project is done as a part of the Nanodegree *Self-Driving Car Engineer* provided by Udacity. The aim of this final project is the application of many parts of the entire nanodegree and the integration on a real self-driving car. After development and testing with the aid of a simulator, the car (Carla) has to cope independently a test track with intersections and traffic lights. The core points of the project are the detection of traffic lights and the integration of functionality for path planning and control in the car by means of the Robot Operating System (ROS) nodes.

![image_screen](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/carla_sdc_1.jpg)


# Outline
1. Team Structure
2. Project Setup
3. Project Description 
4. Architecture and Files
5. Implementation Details
6. Conclusion

# Team Structure
The project is implemented together in a team. 

The following members are part of the international team - Chakra:
- Soheil Jahanshahi (Team Lead)
- Venkata Dikshit
- Daniel Gattringer
- Aneeq Mahmood
- Jongchul Seon
- Wilhelm Nagel

# Project Setup

### Development Setup
The project will require the use of Ubuntu Linux (the operating system of Carla) and a new simulator. Follow the steps below to get set up:
- Because ROS is used, Ubuntu Linux is needed to develop and test the project code.
 - Ubuntu 14.04 with ROS Indigo
 - Ubuntu 16.04 with ROS Kinetic
 - Udacity provides an VM which has ROS and Dataspeed DBW already installed. This VM can be used with [VirtualBox](https://www.virtualbox.org/wiki/Downloads).
 - The system integration project uses a simulator, provided by udacity, which will interface with the ROS code and includes driving on crossroads with traffic lights. To improve the performance while using a VM, running the simulator directly on the host operating system with port forwarding to the guest operating system is recommended. 

### Simulator
- Udacity Term 3 [simulator](https://github.com/udacity/CarND-Capstone/releases) which contains the System Integration part.

### Run the Project
1. Clone the project repository
```bash
git clone https://github.com/solix/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

# Project Description 


# Architecture and Files


# Implementation Details

# Conclusion


---

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
