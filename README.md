This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Purpose

The main goal of the project is to implement some basic functionality of an autonomous vehicle system by writing ROS nodes.
The system architectrue of the used system is shown on the below image:

![system architecture](ros-graph.png)

The project aimed to code the following ROS nodes:
* Traffic Ligth Detection Node
* Waypint Updater Node
* DBW Node

### Team

The project has been completed by the following team.
* Krisztián Sarnyai (krisztian.sarnyai@nng.com)
* Attila Sándor (attila.sandor@nng.com)
* Barnabás Tiegelmann (barnabas.tiegelmann@nng.com)
* Sándor Füleki (sandor.fuleki@nng.com)

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
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

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

### Implementation of the Waypint Updater Node

First we implemented the Waypint Updater node. This node publishes waypoints from the car's actual position to some distance ahead.
For this the node needs to know the position of the car and the list of all the waypoints.
To get this information this node subscribes to the /current_pose and the /base_waypoints topics.
To consider the traffic lights, subscription to the /traffic_waypoint topic is also necessary.

```
	rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
	rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
    
	self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
```

### Implementation of the DBW Node

The DBW (drive by wire) node governs the physical operation of the vehicle by sending throttle, brake, and steering commands.
The input of the node is the /twist_cmd topic, also the /vehicle/dbw_enabled topic is listened to check when we have control of the car.

```
	rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
	rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
	rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
	
	self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
	self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
	self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)	
```	

### Implementation of the Traffic Ligth Detection Node

The purpose of the Traffic Ligth Detection node is to warn the car if there's a red traffic light ahead so the car can stop. The position of all the traffic lights are known via the /vehicle/traffic_lights topic.
Considering the current position of the car (/current_pose) the node should send the index of the waypoint for the nearest upcoming red light's stop line (/traffic_waypoint). 

```
	sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
	sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

	self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
```	

