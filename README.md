# Self driving car system integration project
## Implemented by Team Rushers

#### Dec 15, 2017

# Objective

The objective of the project is to control CARLAs throttle, steering and brakes to navigate map waypoints to successfully drive the car on a highway. The car is expected to follow lanes, traffic lights and objects and plan a trajectory to follow based on the waypoints. The car should be able to classify “Red”, “Green”, and “Yellow” lights and be able to stop/start/slow down based on the traffic signal.
Before running our code on CARLA we developed the code to work in a simulator. 
The simulator works in a very similar way to CARLA as all ROS nodes and topics are same. So if our system works in the simulator, we expect that it should also work on CARLA. 

# The system architecture and principles behind it

The system architecture consists of the following modules:

## Perception

Traffic light detection - We used a deep neural net to detect if the upcoming traffic light is red or not. We trained the classifier once with images from the simulator and once with real images from the ROS bag. A detailed description of the architecture and training parameters can be found in the respective section below (under additional resources implemented) 
We employed the MobileNet architecture to efficiently detect / classify traffic lights. We applied transfer learning … and implemented on two modes as follows: 
•	Simulator mode: classifies whole images as either red/green/yellow. The model was trained with several datasets using the Tensorflow Image Retraining Example 
•	Test-site mode: we employed <> framework to locate a traffic light… 

## Planning
The waypoint updater node publishes a queue of n waypoints ahead of the vehicle position, each with a target velocity. For the simulator, n=100 is sufficient. For the site (the real-world test track), we reduce to n=20. We dequeued traversed waypoints and enqueued new points, preserving and reusing those in the middle. When a light-state changes, the entire queue is updated. The vehicle stops at the final base waypoint. This module is performed using the ROS Package Waypoint updater which is explained as below:
o	Waypoint Updater - sets target velocity for each waypoint based on upcoming traffic lights and obstacles.  This node subscribed to the nodes /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and published a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

## Control subsystems

The control subsystem is implemented using the ROS Package drive-by-wire which adjusts throttle and brakes according to the velocity targets published by the waypoint follower (which is informed by the waypoint updater node). If the list of waypoints contains a series of descending velocity targets, the PID velocity controller (in the twist controller component of DBW) will attempt to match the target velocity
o	DBW (Drive by Wire) - takes target trajectory information as input and sends control commands to navigate the vehicle.  The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node subscribes to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

![alt text](imgs/architecture.png)

## Implementation Nodes

The diagram below illustrates the system architecture. The autonomous vehicle controller is composed of three major units: perception, planning, and control.

![alt text](imgs/Implementation%20Node.png)
  
  a: /camera/image_raw
  b: /current_pose
  c: /current_velocity
  d: /vehicle/dbw_enabled
  e: /traffic_waypoint
  f: /base_waypoints
  g: /final_waypoints
  h: /twist_cmd
  i: /vehicle/throttle_cmd
  j: /vehicle/brake_cmd
  k: /vehicle/steering_cmd

# Operation

There are three modes in which the controller operates:
•	site: When at the test site, this mode is launched. This mode can be run simultaneously with a rosbag to test the traffic light     classifier
•	sitesim: emulates the test site in the simulator at the first traffic light
•	styx: When using the term3 simulator, this mode is launched. The simulator communicates through server.py and bridge.py
These modes are started by roslaunch. For example, to run the styx (simulator) version we run:
roslaunch launch/styx.launch

# Team

•	Boris Dayma, Lead
•	Chris Ferone
•	Taiki Nishime
•	Pabasara Karunanayake
•	Ayanangshu Das

# Results

•	Video from the dash camera onboard the test vehicle:
•	Point cloud visualization:  
•	A map of the test run can be found at <>
•	Log file, ROS bag, and feedback:  
  Below is a visualization of the LIDAR point cloud from the team's test run on the autonomous car. <>

# Additional Resources Implemented

## Traffic Light Image Classification

The perception subsystem dynamically classifies the color of traffic lights in front of the vehicle. In the given simulator and test site environment, the car faces a single traffic light or a set of 3 traffic lights in the same state (green, yellow, red). We assume it is not possible to have multiple traffic lights in the different states at the same time.
We have considered classifying the entire image using CNN approach to solve the traffic light classification task. We used different set of models and got the following results

-Small mobilenet (2Mo): 79.2% test accuracy (664 test samples)
Our program did an exponential moving average on the probability of seeing a red light on each frame and it seemed to be working while it had a test accuracy of about 74%. We decided to use the larger model if it was fast enough; otherwise we just used the smaller model
- Large mobilenet (17Mo): 90.4% test accuracy (664 test samples)

![alt text](imgs/training.jpg)

Considering the fact that traffic lights are always in the same state, and focusing on the creation of a lightweight and fast model, we've chosen the direction of classifying the entire image for the simulator mode of operation. This approach uses a Convolutional Neural Network, which takes a whole image from the front camera as an input and predicts the traffic light state. We used the transfer learning technique on the MobileNet architecture with the Tensorflow Image Retraining Example 

## Dataset

There are multiple datasets, available for model training:
•	images from the Udacity Simulator (images as well as the ground truth from the frontal camera are available as a ROS topic);
https://drive.google.com/open?id=0Bw5abyXVejvMci03bFRueWVXX1U
•	rosbag, captured on the Udacity's test site;
https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view

## Image Preprocessing

On the image pre-processing step we've applied multiple visual transformations:
•	random cropping of the image;
•	rotation on the random angle (+/- 5 degrees);
•	random flipping of the up to 20% images;
•	random color jittering;
Cropping is done by placing a bounding box at a random position in the full image. The cropping parameter controls the size of that box relative to the input image. If it's zero, then the box is the same size as the input and no cropping is performed. If the value is 50%, then the crop box will be half the width and height of the input.

Scaling is a lot like cropping, except that the bounding box is always centered and its size varies randomly within the given range. For example if the scale percentage is zero, then the bounding box is the same size as the input and no scaling is applied. If it's 50%, then the bounding box will be in a random range between half the width and height and full size.

## Neural Network Model

"Simple transfer learning with MobileNet model" example from TensorFlow was used to re-train our model.
We started with a MobileNet model pre-trained on the ImageNet images, and trained a new set of fully-connected layers with dropout, which can recognize our traffic light classes of images. The model works with the image dimensions 224x224x3. The top fully connected layer receives as input a 1001-dimension feature vector (bottleneck_tensor_size = 1001) for each image. 

![alt text](imgs/Network.png)

MobileNets are neural networks constructed for the purpose of running efficiently (high FPS, low memory footprint) on mobile and embedded devices. MobileNets achieve this with 3 techniques:
1.	Perform a depth-wise convolution followed by a 1x1 convolution rather than a standard convolution. The 1x1 convolution is called a pointwise convolution if it's following a depth-wise convolution. The combination of a depth-wise convolution followed by a pointwise convolution is sometimes called a separable depth-wise convolution.
2.	Use a "width multiplier" - reduces the size of the input/output channels, set to a value between 0 and 1.
3.	Use a "resolution multiplier" - reduces the size of the original input, set to a value between 0 and 1

These 3 techniques reduce the size of cumulative parameters and therefore the computation required. Of course, generally models with more parameters achieve a higher accuracy. MobileNets are no silver bullet, while they perform very well larger models will outperform them. MobileNets are designed for mobile devices, NOT cloud GPUs. The reason we're using them in this lab is automotive hardware is closer to mobile or embedded devices than beefy cloud GPUs.

## Accuracy on the simulator data

![alt text](imgs/pasted_image_at_2017_12_13_10_02_am.png)
Figure 1:Depicts Lines on the top are testing/validation of big model (17Mo) and lines at the bottom are testing/validation of small model (2Mo)

## Accuracy on the udacity’s test track data

<>

# Waypoint Updater

## Implementation

Waypoint updater publishes a queue of n waypoints ahead of the vehicle position, each with a target velocity. For the simulator, n=100 should be sufficient. For the site (the real-world test track), we'll need n=20. We dequeued traversed waypoints and enqueued new points, preserving and reusing those in the middle. When a light-state changes, the entire queue may need updating. The vehicle should stop at the final base waypoint in the simulator (the default provided by the waypoint loader). This is overridden for the test-site, enabling the vehicle to loop around the track repeatedly.
Initialization of queue:
•	Create a persisted queue
•	Locate closest waypoint (in the base waypoint list) ahead of current position of vehicle and enqueued n waypoints.
•	Persist the index of the last retrieved point in the base list.
Operation cycle at a frequency of 2 Hz:

Operation cycle at a frequency of 2 Hz:

•	Manage waypoint queue: Dequeue consumed waypoints with coordinates "behind" the current vehicle position. Enqueue waypoints from base list starting from last until queue is restored to n in length. (Unless final is encountered).
•	Update velocity: If red/yellow light NOT within range (4 m for site, 62 m for simulator), set waypoint velocity to defaults (10 km/h for site, 40 km/h for simulator) given in base list. If red/yellow within range, update each waypoint velocity so that the vehicle comes to a halt at the stop-line waypoint. Decrease velocity at a constant rate of -1.0 m/s2. Velocity beyond the stop line should be set to zero.

## Optimization

As the vehicle moved along a path, the waypoint updater made changes to the planned path. Presently, there are no dynamic obstacles other than the traffic lights. In theory, we can make a plan and then simply follow that plan, add to the end of that plan, and make adjustments when a traffic light changes state.
Currently, the code creates a completely new list of waypoints on each cycle. (This is inefficient...as an initial draft.) This list contains the next <> waypoints from our present position. The re-planning event occurs at a rate of once per pose callback (<> times per second). 
Ideally this planning can occur far less frequently. For example only when a change in traffic light is detected or as we near the end of our current waypoint list. Alternatively we might add a few new waypoints to the end of our current list (and avoid re-creating the entire list).
An optimized updater is important to the Drive-By-Wire node because it will reduce unnecessary accelerations due to plan changes. For instance, when the vehicle is approaching a red traffic light, a ramp down in velocity is planned. If the one plan is followed, the acceleration will be smoothly executed. On the other hand, presently the updater replans too frequently, and because the vehicle is in a new location, a new velocity ramp is created. This leads to interruptions in the deceleration plan to the stop line. 

# Drive By Wire

## Velocity Targets (Stepped)

The drive-by-wire node adjusts throttle and brakes according to the velocity targets published by the waypoint follower (which is informed by the waypoint updater node). If the list of waypoints contains a series of descending velocity targets, the PID velocity controller (in the twist controller component of DBW) will attempt to match the target velocity. 
In the chart below, for the chart with title speed, we transition from 13 mps to 0 mps through a series of decreasing velocities across a plurality of waypoints. The blue line represents the target velocity, green is actual velocity. Green is braking on a scale from 0 to 1. The x-axis is the number of frames at 50 FPS (rate = rospy.Rate (50)).

![alt text](imgs/figure_2-2.png)


# The original readme from Udacity

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here.](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9)

## Native Installation

   * Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found     here.](https://www.ubuntu.com/download/desktop)

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

   * [Download the Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

## Docker Installation

[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container

    Docker build . -t capstone

Run the docker file

    docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone

## Usage

Clone the project repository

    git clone https://github.com/udacity/CarND-Capstone.git

Install python dependencies

    cd CarND-Capstone
    pip install -r requirements.txt
    
Make and run styx

    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch
    
Run the simulator

## Real world testing

A. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))

B. Unzip the file

    unzip traffic_light_bag_files.zip
    
C. Play the bag file

    rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
    
D. Launch your project in site mode

    cd CarND-Capstone/ros
    roslaunch launch/site.launch
    
E. Confirm that traffic light detection works on real life images
