# Introduction
This project is done as a part of the Nanodegree *Self-Driving Car Engineer* provided by Udacity. The aim of this final project is the application of many parts of the entire nanodegree and the integration on a real self-driving car. After development and testing with the aid of a simulator, the car (*Carla*) has to cope independently a test track with intersections and traffic lights. The core points of the project are the detection of traffic lights and the integration of functionality for path planning and control in the car by means of the Robot Operating System ([ROS](http://www.ros.org/)) nodes.

The self driving car *Carla*:
![image_screen](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/carla_sdc_1.jpg)

Testing the system in the simulator:
![sim_screen](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/screen_simulator.jpg)

# Outline
1. Team Structure
2. Project Setup
3. Project Description 
4. ROS Architecture and Nodes
5. ROS Topics
6. ROS Message Types
7. Implementation Details
8. Conclusion

# Team Structure
The project is implemented together in a team. 

The following members are part of the international team named ***Chakra***:
* **Team Lead**
  * Soheil Jahanshahi ([soheil.jahanshahi@gmail.com](mailto:soheil.jahanshahi@gmail.com))
* **Team Members**
  * Venkata Dikshit ([dikshit2632@gmail.com](mailto:dikshit2632@gmail.com))
  * Daniel Gattringer ([daniel@gattringer.biz](mailto:daniel@gattringer.biz))
  * Aneeq Mahmood ([aneeq.sdc@gmail.com](mailto:aneeq.sdc@gmail.com))
  * Wilhelm Nagel ([willi.nagel@gmail.com](mailto:willi.nagel@gmail.com))

# Project Setup

### Development Setup
The project will require the use of Ubuntu Linux (the operating system of *Carla*) and a new simulator. Follow the steps below to get set up:
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
### Project Goals
The goal of the project is that an autonomous car - in a simulator or as a real vehicle - can handle a given test track without the intervention of a safety driver. The vehicle shall follow a given path, comply with the speed limit, accelerate and steer in a comfort-oriented manner without exceeding the limits for acceleration and jerk, and finally stop at red traffic lights and continue at green traffic lights.

### Simulator
For development and testing, a simulator is used. This simulator developed and made available by Udacity is based on the open source game engine [Unity 3D](https://unity3d.com).

### The self-driving Car - *Carla* 
*Carla* is an autonomous Lincoln MKZ, running on the udacity test site in Palo Alto, California.

*Carlas* Hardware Specs:
* 31.4 GiB Memory
* Intel Core i7-6700K CPU @ 4 GHz x 8
* TITAN X Graphics
* 64-bit OS

# ROS Architecture and Nodes
*Carla* uses Robot Operating System ([ROS](http://www.ros.org/)) to integrate all main functionalities.

The following images shows the main ROS architucture for the project:
![architecture](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/final-project-ros-graph-v2.png)

### Traffic Light Detection Node
This node takes the image stream from the car's camera from `/image_color`, the current position of the car from `/current_pose)`, and a list of waypoints around the track from `/base_waypoints` and finally publishes the locations to stop for red traffic lights to the `/traffic_waypoint`. Only the index of the waypoint, which is nearest to the stop line of a red light, is published. So the control system of the car knows where it has to stop and can brake smoothly.

![image_traffic_light_detection](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/tl-detector-ros-graph.png)

### Waypoint Updater Node
This node takes the current position of the car from `/current_pose)`, and a list of waypoints around the track from `/base_waypoints` and finally publishes the next waypoints the car should follow to `/final_waypoints`. The number of waypoints to look ahead is defined by `LOOKAHEAD_WPS`, which is configured to `50`(TODO: final number). Within the node, the next waypoints are selected, which are in front of the car and provided with target speeds to allow a smooth movement and a targeted braking and accelerations at traffic lights.

![image_waypoint_updater](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/waypoint-updater-ros-graph.png)

### DBW Node
This node takes the current velocity of the car from `/current_velocity)`, the information if the control system should maneuver the car or a safety driver does from `/vehicle/dbw_enabled`, and twist commands from `/twist_cmd` and finally publishes commands for *Carla's* drive-by-wire system to `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd`. The twist commands are generated and published to `/twist_cmd` by the waypoint follower node, which uses the data from `/final_waypoints` to generate this commands.

![image_dbw](https://github.com/solix/CarND-Capstone/blob/master/info_for_readme/dbw-node-ros-graph.png)

### Additional Nodes
* styx
  * This package contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* styx_msgs
  * This package includes definitions of the custom ROS message types used in the project.
* waypoint_loader
  * This package loads the static waypoint data and publishes to `/base_waypoints`.
* waypoint_follower
  * This package contains code from Autoware which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

# ROS Topics

### Waypoints
* /base_waypoints
  * Provides a message of custom type `styx_msgs/Lane` which contain waypoints as provided by a static .csv file.
* /obstacle_waypoints
  * Not used at the moment
* /traffic_waypoint
  * Provides a message which contains the **index** of the waypoint for nearest upcoming red light's stop line.
* /final_waypoints
  * Provides a message of custom type `styx_msgs/Lane` which contain a subset of `/base_waypoints`. The first waypoint is the one in `/base_waypoints` which is closest to the car (in front of it).

### Vehicle Data/Status
* /image_color
  * Provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights
* /current_pose
  * Provides messages of custom type `geometry_msgs/PoseStamped` which contain the current position of the vehicle, delivered by the simulator or the localization module of *Carla*.
* /current_velocity
  * Provides messages which contain the current velocity of the vehicle, delivered by the simulator or localization module of *Carla*.
* /vehicle/dbw_enabled
  * Provides the information if the control system should maneuver the car or a safety driver does. This information is available in both cases - using the simluator and using *Carla*.

### Vehicle Control
* /twist_cmd
  * Provides a message which contains the vehicle linear and angular velocities. (TODO: of the next waypoint?)

### Controller -> Car/Simulator
* /vehicle/brake_cmd
  * Published brake values are in units of torque `(N*m)`. The values for brake are computed by using the desired acceleration, the weight of the vehicle, and the wheel radius.
* /vehicle/steering_cmd
  * TODO find out which values are published here.
* /vehicle/throttle_cmd
  * Published throttle values are in the range 0 to 1.

# ROS Message Types

In addition to the [standard ROS message types](http://wiki.ros.org/std_msgs) like Int32, the following custom messages were used:

* Lane.msg
```
Header header
Waypoint[] waypoints
```

* Waypoint.msg
```
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
```

* TrafficLight.msg
```
Header header
geometry_msgs/PoseStamped pose
uint8 state

uint8 UNKNOWN=4
uint8 GREEN=2
uint8 YELLOW=1
uint8 RED=0
```

* TrafficLightArray.msg
```
Header header
TrafficLight[] lights
```

# Implementation Details

### Perception

### Planning

### Control



# Conclusion

TODO
