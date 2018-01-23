# Capstone Project Starter Code

The goal of this project is the integration of all systems that manage the real driving of a vehicle (Carla) using previously a simulator which will interface with your ROS code and has traffic light detection.
<!--more-->

[//]: # (Image References)

[image1]: /imgs/ros-graph-v2.png "ROS System"
[image2]: /imgs/tl-detector.png "Traffic light detection node"
[image3]: /imgs/waypoint-updater.png "Waypoint updater node"
[image4]: /imgs/dbw-node.png "Dbw node"
[image5]: /imgs/squeezenet.png "Squeezenet"
[image6]: /imgs/MobileNet-SSD.png "MobileNet-SSD"
[image7]: /imgs/Red_detec.png "Red light"
[image8]: /imgs/Yellow_detec.png "Yellow light"
[image9]: /imgs/Green_detec.png "Green light"
[image10]: /imgs/None_detec.png "Nothing"

#### TEAM SKYNET

Team members is provided in the table below:

| Full Name                      | Slack      | Email                                                                                  |
| :----------------------------- | :--------- | :------------------------- |
| Dennis Korotyaev (Team Leader) | @4skynet   | nemiroff.den@gmail.com     |
| Samip Shah                     | @shahsamip | samipshah86@gmail.com      |
| Abhay Carande Luna             | @abhaycl   | abhaycl@hotmail.com        |
| Marcelo Nascimento             | @mzumbin   | mzumbin@gmail.com          |

#### How to run the program with the simulator

**Note:** The file used as a detection model must be unzipped in the same folder because it has a large size, it's in the path: 

  **/ros/src/tl_detector/model/model_detection.zip**

should look like this:

  **/ros/src/tl_detector/model/model_detection.rb**

---

Make and run styx.
```bash
1.  cd ros
2.  catkin_make
3.  source devel/setup.sh
4.  roslaunch launch/styx.launch
```
Run the simulator.

---

The summary of the files and folders int repo is provided in the table below:

| File/Folder                     | Definition                                                                                            |
| :------------------------------ | :---------------------------------------------------------------------------------------------------- |
| data/*                          | Folder that contains the data used by the nodes.                                                      |
| imgs/*                          | Folder that contains the images to visualize.                                                         |
| ros/*                           | Folder that contains all project source files.                                                        |
| ros/src/tl_detector/            | It contains everything related to the traffic light detection node.                                   |
| ros/src/waypoint_updater/       | It contains everything related to the Waypoint updater node.                                          |
| ros/src/twist_controller/       | It contains everything related to the Dbw node.                                                       |
| ros/src/styx/                   | It contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics. |
| ros/src/styx_msgs/              | It contains the definitions of the custom ROS message types used in the project.                      |
| ros/src/waypoint_loader/        | It contains the loads the static waypoint data and publishes to /base_waypoints.                      |
| ros/src/waypoint_follower/      | It contains code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic. |
|                                 |                                                                                                       |
| Dockerfile                      | Add image-proc package to Dockerfile.                                                           |
| requirements.txt                | Contains the correct versions to use with practice according to the architecture used in the vehicle. |

---
The specifications and necessary requirements are detailed below, it is the documentation provided in the Capstone repository by Udacity for the correct realization of the final practice.

### Ros System

The ROS system used by the simulator and the actual vehicle is:

![Final score][image1]

The ROS system is composed mainly of the following sections:

  * Traffic light detection node.
  * Waypoint updater node.
  * Dbw node.

#### Traffic light detection node

![Final score][image2]

The classification model used is [SqueezeNet](https://arxiv.org/pdf/1602.07360.pdf) which is composed as shown in the image below:

![Final score][image5]

We have used the MS COCO dataset class as a pre-trainer [ssd_mobilenet_v1_coco](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz) from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md, only class 10 is needed for traffic lights.

The ROS traffic light detector is implemented in the node *tl_detector* in the classes *TLDetector* and *TLClassifier*. *TLDetector* is responsible for finding the nearest traffic light location and calls *TLClassifier.get_classification* with the current camera image.

*TLClassifier* initially uses the MobileNet-SSD model to detect a traffic light delimiter box with the maximum score. If the delimiter box is found, the cropped traffic light image adapts to a 32x32 image and the SqueezeNet model changes the traffic light color (red, yellow, green). If at least the last 3 images were classified as red, then TLDetector publishes the index of the traffic light waypoint in /traffic_waypoint.

![Final score][image6]

Detection of traffic lights by camera:

![Final score][image7] ![Final score][image8] ![Final score][image9] ![Final score][image10]

#### Waypoint updater node

![Final score][image3]

Waypoint updater publishes the next 200 waypoints ahead of the car position, with the velocity that the car needs to have at that point. Each 1/20 seconds, it does:

  * Update of closest waypoint. It does a local search from current waypoint until it finds a local minimum in the distance. If the local minimum is not near (less than 20m) then it assumes it has lost track and does perform a global search on the whole waypoint list.

  * Update of velocity. If there is a red ligth ahead, it updates waypoint velocities so that the car stops ~stop_distance (node parameter, default: 5 m) meters behind the red light waypoint. Waypoint velocities before the stop point are updated considering a constant ~target_brake_accel (default: -1.0 m/s^2).

Besides, the car is forced to stop at the last waypoint if either its velocity in /base_waypoints is set to 0 or the parameter ~force_stop_on_last_waypoint is true.

#### Dbw node

![Final score][image4]

The Drive By Wire node is responsible for controlling the following elements:

  * **Steering.-** It's controlled by a combination of predictive and corrective steering.
               Predictive Steering is implemented using the class provided by YawController
               and the corrective direction is calculated with the cross path error,
               which is passed to a linear PID that returns the correct direction angle.
               These values are added together to obtain the final turning angle.

  * **Throttle.-** It's controlled by the linear PID when an error occurs in speed,
               it's the difference between the current speed and the proposed speed.

  * **Brake.-** If a negative value is returned by the throttle PID, it means the car needs
               to slow down or brake. The braking torque is calculated taking into account
               vehicle mass, fuel capacity, gas density, wheel radius and deceleration.

---
### Documentation Provided in the Capstone Repository

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

Yes
