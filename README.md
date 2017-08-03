### Installation

* [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)

Download the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim).

### Usage

1. Install python dependencies
```bash
cd styx
pip install -r requirements.txt
```
2. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
3. In terminal, specify path to Unity simulator
```bash
What is the full path to your Unity simulator?
/home/calebkirksey/Desktop/lights_no_auto_no_cars/ros_test.x86_64
```
The virtual car should start moving in the Unity simulator
![unity running](imgs/unity.png)
