# Introduction
This project is done as a part of the Nanodegree *Self-Driving Car Engineer* provided by Udacity. The aim of this final project is the application of many parts of the entire nanodegree and the integration on a real self-driving car. After development and testing with the aid of a simulator, the car (Carla) has to cope independently a test track with intersections and traffic lights. The core points of the project are the detection of traffic lights and the integration of functionality for path planning and control in the car by means of the Robot Operating System (ROS) nodes.

The self driving car Carla:
![image_screen](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/carla_sdc_1.jpg)

Testing the system in the simulator:
![sim_screen](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/screen_simulator.jpg)

# Outline
1. Team Structure
2. Project Setup
3. Project Description 
4. ROS Architecture and Nodes
5. ROS Topics
6. Implementation Details
7. Conclusion

# Team Structure
The project is implemented together in a team. 

The following members are part of the international team - Chakra:
* Soheil Jahanshahi (Team Lead)
* Venkata Dikshit
* Daniel Gattringer
* Aneeq Mahmood
* Jongchul Seon
* Wilhelm Nagel

# Project Setup

### Development Setup
The project will require the use of Ubuntu Linux (the operating system of Carla) and a new simulator. Follow the steps below to get set up:
* Because ROS is used, Ubuntu Linux is needed to develop and test the project code.
  * Ubuntu 14.04 with ROS Indigo
  * Ubuntu 16.04 with ROS Kinetic
  * Udacity provides an VM which has ROS and Dataspeed DBW already installed. This VM can be used with [VirtualBox](https://www.virtualbox.org/wiki/Downloads).
* The system integration project uses a simulator, provided by udacity, which will interface with the ROS code and includes driving on crossroads with traffic lights. To improve the performance while using a VM, running the simulator directly on the host operating system with port forwarding to the guest operating system is recommended. 

### Simulator
* Udacity Term 3 [simulator](https://github.com/udacity/CarND-Capstone/releases) which contains the System Integration part.

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


### Car/Simulator 
The real car (Carla) is an autonomous Lincoln MKZ, running on the udacity test site in Palo Alto, California.

Carlas Hardware Specs:
* 31.4 GiB Memory
* Intel Core i7-6700K CPU @ 4 GHz x 8
* TITAN X Graphics
* 64-bit OS

# ROS Architecture and Nodes
Carla uses Robot Operating System ([ROS](http://www.ros.org/)) to integrate all main functionalities.

The following images shows the main ROS architucture for the project:
![architecture](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/final-project-ros-graph-v2.png)

### Traffic Light Detection Node
TODO
![image_traffic_light_detection](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/tl-detector-ros-graph.png)

### Waypoint Updater Node
TODO
![image_waypoint_updater](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/waypoint-updater-ros-graph.png)

### DBW Node
TODO
![image_dbw](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/dbw-node-ros-graph.png)

### Additional Nodes
* styx
  * This package contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* styx_msgs
  * This package includes definitions of the custom ROS message types used in the project.
* waypoint_loader
  * This package loads the static waypoint data and publishes to /base_waypoints.
* waypoint_follower
  * This package contains code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic.

# ROS Topics

### Waypoints
* /base_waypoints
  * TODO
* /obstacle_waypoints
  * TODO
* /traffic_waypoint
  * TODO
* /final_waypoints
  * TODO

### Vehicle Data/Status
* /image_color
  * TODO
* /current_pose
  * TODO
* /current_velocity
  * TODO
* /vehicle/dbw_enabled
  * TODO

### Vehicle Control
* /twist_cmd
  * TODO

### Controller -> Car/Simulator
* /vehicle/brake_cmd
  * TODO
* /vehicle/steering_cmd
  * TODO
* /vehicle/trottle_cmd


# Implementation Details

### Perception

### Planning

### Control



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
