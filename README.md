This is the project repo for the Autonomous Wizards team from the *inaugural* (July 7th - October 16th, 2017) cohort of the Udacity Self Driving Car Engineer Nanodegree final & capstone project for said Nanodegree, alternately titled "System Integration" & "Programming a Real Self Driving Car".  For more information about the project, see the project introduction [here--note: **massive** "paywall", you have to be registered for the 3rd Term of said Nanodegree, total cost for said activity being $2400, plus having passed the prior two terms / 10 projects](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

A video showing a complete run of the virtual track in the simulator by our current--as of October 10th, 2017--version of our repo can be found here, 
at this [link to YouTube video--or just \<ctrl\>click (to open in a new tab) on thumbail below, same link](https://youtu.be/zUsNETAbcLU) 

[![Autonomous Wizards lap in Carla-Simulator](video/Screenshot%202017-10-10%2010:00:31.png)](https://youtu.be/zUsNETAbcLU) 

Introducing Team Autonomous Wizards (Members in alphabetical order):

Juan Carlos Ortiz ortizjuan2@gmail.com 

Chuck S. chuck_s_@outlook.com 

Ezra J. Schroeder ezra.schroeder@gmail.com 

Christian Sousa neocsr@gmail.com 

Calvenn Tsuu calvenn.tsuu@gmail.com 

### Documenting code 

![alt text](ezra_linear_algebra_2.jpeg)


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
git clone https://github.com/seneca-wolf/CarND-Capstone 
```

2. Install python dependencies (Please note: if you do not have ROS installed / experience w/ ROS 
it may interfere w/ your [e.g. conda] python distributions & environments, which is why Udacity uses 
a specific preconfigured Virtual Machine for the Capstone project). Also, please note that the setup 
here (particularly in item 3, below) are different than for the original Udacity repo for this project.
C.f. the changelog.txt file maintained by Chuck. 
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
cd src 
catkin_init_workspace
cd .. 
catkin_make
source devel/setup.bash 
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
