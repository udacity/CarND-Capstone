# Project Team

  * Reinhard Steffens steffens21@gmail.com
  * Yury Melnikov Yury.Melnikov@gmail.com
  * Naoto Yoshida yossy0157@gmail.com
  * Tawit Uthaicharoenpong iamtawit@gmail.com
  * Olli Vertanen overtane@gmail.com

## Notes on our Implementation

### Traffic Light Detection

The traffic light detection is done by using a Tensorflow model Faster R-CNN + ResNet101 trained on the COCO dataset which also includes traffic lights: [faster\_rcnn\_resnet101\_coco](http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz). The output of this model is a series of bounding boxes, which are screened for traffic signals of a high score (>10%).

Classification of traffic signals is implemented in this way:
1) Threshold luma of each third of the signal image resulting in a binary image.
2) Add binary of the red color detected in the HSV color space of the first third of the signal
   image.
3) Add binary of the yellow color of the second third.
4) Add binary of the green color of the last third.
5) Pick the light whose average weight is higher than 0.1 and at least twice higher than the second greatest
   weight.

The traffic light detector gets stop line positions for the traffic lights from the route configuration, and the waypoints of the route from the `/base_waypoints` ROS topic. It uses this information to find out a stop point for each traffic light along the route. A stop point is a waypoint on which the vehicle should stop in case of red light.

![tl detection](./videos/tl_detection.gif)

### Waypoint Updater
After the traffic light detection was working sufficiently, we used the traffic light position and its status to adjust the vehicle target velocities.

The vehicle's target velocty is adjusted from the original waypoint velocity when a traffic light is closer than 50 meters ahead of the car and the status of the traffic light is either 'RED' or 'YELLOW'. Once these condition are met, the target velocities for each waypoint will be linearly decreased down to a complete stop at the light.

The waypoint updater publishes to the ```/next_waypoint_ahead``` topic, which contains the next upcoming waypoint for the vehicle. 

### Twist Cotroller
* Throttle Control : PID control
  * We utilized the PID controller (```pid.py```) and the low-pass filter class (```lowpass.py```, we rewrite LPF for the clarity) in the throttle control . We calculated the error (```velocity_diff```) between the target velocity (```twist_cmd.linear.x```) and the current velocity (```current_velocity.linear.x```). Then we obtained the reactive throttle signal from this error signal. Because raw throttle outputs were somewhat jaggy, we filtered the throttle signal by a LPF to smooth the final output. We hand-tuned PID and LPF parameters by actually running the controller in the simulator (```twist_controller.py```).
* Steering Control : PID control
  * We used a PID controller for the steering control as well. In this case we set the target of the car's angular_velocity always zero. We used ```angular_velocity (twist_cmd.angular.z)``` as the error signal and then applied a PID controller and a LPF as just like in the throttle control.
* Brake Control : Torque control
  * The brake control is enabled instead of the throttle control when (1) the target velocity is decreasing and the difference between the target velocity and the current velocity (```velocity_diff = linear_velocity - current_velocity```) is positive and less than the threshold value (1.0 in our implementation) OR (2) the target velocity is smaller than the threshold (```brake_deadband```).
  * From the requirement of the brake controller, we calculated the total brake torque by the multiplication of the car's total mass (```self.total_mass.```), wheel radius (```self.wheel_radius```), the required deacceleration (```velocity_diff/time_interval```) and a brake constant as a tuning parameter. If the deacceleration is too small, the car cannot stop completely. In order to ensure the stop at red traffic lights, we took the larger one from the required acceleration or a constant.

![twist control](./videos/twist_controller.gif)


***

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
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
