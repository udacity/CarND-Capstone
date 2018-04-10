# Team FusionX - CarND Capstone Project

Udacity Self-Driving Car Nanodegree, 2018

### Team Members

* Effendi Dufford
* [Taylor Raack](https://github.com/dinoboy197)
* Meenu Natarajan
* Anthony Knight
* Shripad Kondra

### Project Overview

The **final capstone project** of the Udacity Self-Driving Car Nanodegree program provides the opportunity to run our code on a modified Lincoln MKZ hybrid vehicle (named "Carla") to **autonomously drive around a test lot course** with a set of mapped route waypoints and a traffic light.

The vehicle has a [Dataspeed](http://dataspeedinc.com/) **drive-by-wire (DBW)** interface for throttle/brake/steering control, a **forward-facing camera** for traffic light detection, and **LIDAR** for localization (processed by Udacity to provide the car's current pose).  The code is run on a Linux PC with **Robot Operating System** ([ROS](http://www.ros.org/)) and a TitanX GPU for **TensorFlow** processing.

Since the team members are working from **multiple global locations (US, Canada, India)**, all development has been done online through **Slack and Github team collaboration** while using the [Udacity vehicle simulator](https://github.com/udacity/CarND-Capstone/releases) to integrate and prove out the algorithms.  After being able to drive the simulator courses, the code is then tested on the real vehicle by Udacity engineers in California.

### System Architecture

[<img src="./imgs/final-project-ros-graph-v2.png" width="900">](https://github.com/team-fusionx/CarND-Capstone/blob/master/imgs/final-project-ros-graph-v2.png)

*Image source: Udacity Project Overview lesson.  Note: Obstacle Detection was not part of this project.*

The autonomous control system architecture starts by **loading mapped route base waypoints** in the Planning area's Waypoint Loader and setting an overall **max speed guard** for each waypoint.  This initial setup was provided by Udacity, to protect for safe operation in the test lot.

The system then starts **receiving the car's sensor data** (current pose from LIDAR localization, current speed, DBW enable switch, and camera image).

The Perception area's **Traffic Light Detection Node** processes the camera image to **detect traffic lights** to decide if and **where the car needs to stop** at an upcoming waypoint location.

The Planning area's **Waypoint Updater Node** plans the driving path target speed profile by **setting upcoming waypoints with associated target speeds**, including smoothly accelerating up to the target max speed and slowing down to stop at detected red lights.

The Control area's **Waypoint Follower** sets **target linear velocity** (from the planned waypoint target speeds) and **target angular velocity** (using Autoware's Pure Pursuit library algorithm to steer toward the waypoint path).

The Control area's **DBW Node (Twist Controller)** sets the **throttle, brake, and steering commands** using PID feedback control for throttle and brake, and kinematic bicycle model yaw control for steering.  These commands are sent to the Dataspeed DBW system to actuate the car's pedals and steering wheel.

| Area        | Task                     | Primary Member | Secondary Member | Description                                                                                                                                                                                             |
|:-----------:|:------------------------:|:--------------:|:----------------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| Perception  | Traffic Light Detection  | Shripad        | Meenu            | Train/implement neural network traffic light classifier, and determine the stopping locations for red lights                                                                                            |
| Planning    | Waypoint Loader          | -              | -                | Use Udacity provided base code                                                                                                                                                                          |
| Planning    | Waypoint Updater         | Anthony        | Effendi          | Design and implement a smooth speed profile planner using [Jerk Minimizing Trajectory (JMT)](http://courses.shadmehrlab.org/Shortcourse/minimumjerk.pdf) following dynamic red light stopping locations | 
| Control     | Waypoint Follower        | Effendi        | Taylor           | Implement improvements to Autoware's base Pure Pursuit library to set target linear velocity and target angular velocity to follow upcoming waypoints                                                   |
| Control     | [DBW (Twist Controller)](dbw_control.md)   | Taylor         | Effendi          | Implement & tune PID feedback control with low pass filtering for throttle/brake commands and kinematic yaw control for steering command                                                                |
| Integration | Simulation Testing       | Meenu          | Anthony          | Test & debug fully integrated control system with the simulation on a highway track and test lot course                                                                                                 |
| Integration | Real-world Image Testing | Meenu          | Shripad          | Test & debug traffic light classifier with real-world camera images from recorded ROS bag data                                                                                                          |
| Integration | Visualization Tools      | Effendi        | Taylor           | Set up data visualization & analysis tools using ROS RQT with Multiplot plugin and RViz 3D scene viewer                                                                                                 |

### Implementation Details

#### Perception

* [Traffic Light Detection Node](https://github.com/team-fusionx/CarND-Capstone/wiki/Traffic-Light-Detection)

#### Planning

* [Waypoint Updater Node](https://github.com/team-fusionx/CarND-Capstone/wiki/Waypoint-Updater)

#### Control

* [Waypoint Follower](https://github.com/team-fusionx/CarND-Capstone/wiki/Waypoint-Follower)
* [DBW Node (Twist Controller)](dbw_control.md)

#### Integration

* [Visualization Tools](https://github.com/team-fusionx/CarND-Capstone/wiki/Visualization-Tools)

---

### *Original setup instructions from Udacity base repo:*

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
