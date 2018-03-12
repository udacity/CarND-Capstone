# Self-Driving Car Engineer Nanodegree

## Capstone Project - System Integration

As the Capstone Project of the Self-Driving Car Engineer Nanodegree, the System Integration project aims at integrating three key modules, Perception, Planning and Control, into a complete solution to the Autonomous Driving. The final code will be tested on Carla, the Udacity’s Self Driving Lincoln MKZ, around a test track in Califonia.

### Team: Speedy Easter Bunny

**Peng Xu**

**Ravel Antunes**

**William O Grady**

**Danilo Canivel**

**Naveen Pandey**

### Architecture

### Perception

#### 1. Traffic Light Detector Node

The purpose of the traffic light detector node is to publish the waypoint location and state (colour of nearest traffic light) at that time. This node calls the traffic light classification algorithm and passes in the current frame in order to determine the colour of the lights. 

The node was implemented with the following algorithm:

1. The closest lights to the ego vehicle is identified, its waypoint index index is recorded based on its position.
2. Once the nearest light is located, the nearest stop line to the traffic light is found. As before the resulting data is the waypoint index
3. Next step after the traffic lights and stop line way points are found is to call the Traffic Light Classification algorithm (see below) and determine the colour of the lights
4. Once the ego vehicle is close enough to the lights we report the current colour and stop line waypoint index.

#### 2. Traffic Light Classification

Multiple approaches were investigated to determine the colour of the lights. These approaches first included the use of a SVM classifier and a GCForest classifier but finally ended with using inference based ssd_inception trained model. This model was based on the following [blog](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)

The data collection stage itself took a bit of time as data from both the simulator and real world was required. Note also 2 models were trained, one for real world testing and the other for the simulator.

# WIP

### Planning

#### 1. Waypoint Loader Node

#### 2. Waypoint Updater Node

### Control

#### 1. Waypoint Follower Node

#### 2. Drive By Wire (DBW) Node

##### 1) Throttle Controller

##### 2) Steering Controller

##### 3) Braking Controller

### Test on Simulator

### Test on Carla

### Conclusion

------------------
(Below are original readme.)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

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

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

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
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
