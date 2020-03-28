Team Name: (any suggestions?)

Team Members: 
| Name        | email           |Task|
| :-------------: |:-------------:|:-------------:|
|Tianji Li      	|     ltjsun@126.com|Implement waypoint updater|
| Xiao Chen	      |     xiao.chen@student.kit.edu|Integrate the model in the ros code/ Readme|
| Bhavesh Parkhe	 |     bparkhe@umass.edu|debug and test code|
| Hanyu Wu       	|     wuh199410@gmail.com|Labeling and Train the Object detection model|
| Arnab Dutta	    |     arnab.dutta@daimler.com|Implement waypoint updater and dbw_node/ manage Github|

Team Lead: Arnab Dutta

* The training set (jpeg images) pulled from the simulation can be found [here](https://drive.google.com/drive/folders/11l1uY78W-bPMJTJQtnuVBoWa5jCoAsby?usp=sharing).
* The training set used for the on-site detector training can be found [here](https://drive.google.com/drive/folders/1JmIMDg9kN88769HYZZlic6ecOI40NGBg?usp=sharing).


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Implementation
* Code Structure
![ros_structure](/imgs/ros_structure.png)
* TODO Lists:
  * waypoint_updater: This node will publish waypoints from the car's current position to some 'x' distance ahead
  * dbw_node: **drive-by-wire node**, which will subscribe to **twist_cmd** and use various controllers to provide appropriate throttle, brake, and steering commands. These commands can then be published to the following topics:
    * `/vehicle/throttle_cmd`
    * `/vehicle/brake_cmd`
    * `/vehicle/steering_cmd`
  * twist_controller: contains the **Controller** class. The **control** methode can take **twist data** as input and return **trottle**, **control**, and **steering values**
  * tl_detector: **The traffic light detection node**
    * Use the vehicle's location and the (x, y) coordinates for traffic lights to find the nearest visible traffic light ahead of the vehicle. We can use ground truth to test the other parts without detection: in methode **get_light_state**, `return light.state`
    
    * Use the camera image data to classify the color of the traffic light.
    
  * tl_classifier: Take the **BGR Image** as input, output the **ID of traffic light color** (specified in `styx_msgs/TrafficLight`) <br>We used the [Object Detection Lab]( https://github.com/udacity/CarND-Object-Detection-Lab) and replaced the pb-file with our pretrained model file (Training Process will be described below) [frozen_inference_graph_sim.pb](/src/ros/tl_detector/light_classification/frozen_inference_graph_sim.pb) for simulation and for the on-site test we will implemnet a different model [frozen_inference_graph_site.pb](/src/ros/tl_detector/light_classification/frozen_inference_graph_site.pb).<br>
  * Object detection model training: We use [MobileNet SSD](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) to detect the differenr traffic light signals. It is accurate and fast, takes just about 60ms to process a frame using a laptop's CPU (Intel(R) Core(TM) i5-8265U CPU @ 1.60GHz). We trained each model for 200k steps (batch size = 24) on google cloud platform using NVIDIA® Tesla® K80 based on the pretrained COCO model. The mean Average Precision in simulation task can be over 0.95 at 0.50IOU, and over 0.88 at 0.50IOU in on-site task. In fact we first trained the on-site detector based on the [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132), but it behaved very poor on the on-site images, then we directly turn to the on-site images (very small dataset).

### Results
* **Test video of Simulation**, click the figure below:<br>[![ScreenShot](/imgs/test_sim.png)](https://www.youtube.com/watch?v=m2vrWBcxkZs&feature=youtu.be)<br>

* A snippet of the on-site traffic light detection is shown here:<br>
<img src="imgs/on-site-snippet.png">

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

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.