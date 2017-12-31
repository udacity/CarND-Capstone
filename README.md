## Capstone Project for Carla

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team Members

|   Name                            |   Udacity account email            |
|-----------------------------------|------------------------------------|
| John Moody (**Team Lead**)        | John.Moody at ieee.org             |
| Jayasim Jayakumar                 | jayasim at gmail.com               |
| Rajeev Ranjan                     | rajeev.cse.imps at gmail.com       |
| Sven Bone                         | sven.bone at mail.de               |
| Bassam Sayed			            | b`underscore`sayed at icloud.com	 |



### How DBW Works?
We have created a TwistController class from twist_controller.py which will be used for implementing the necessary controllers. The throttle values passed to publish should be in the range 0 to 1, although a throttle of 1 means the vehicle throttle will be fully engaged. Brake values passed to publish should be in units of torque (N*m). The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

Here we will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities. Yawcontroller is used to convert target linear and angular velocity to steering commands. `/current_velocity` provides the velocity of the vehicle from simulator. 

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

Once we have the proposed throttle, brake, and steer values, we published it on the various publishers.
We have currently set up to publish steering, throttle, and brake commands at 50hz.  The DBW system on Carla expects messages at this frequency, and will disengage (reverting control back to the driver) if control messages are published at less than 10hz. This is a safety feature on the car intended to return control to the driver if the software system crashes.

### Export Images from Simulator and ROS Bags

The image export node `tl_image_extractor.py` can be configured by its launch files. There are basically two launch files, one for 
the simulator setup `tl_image_extractor.launch` and one for the rosbag setup `tl_image_extractor_site.launch`. 

**Attention:**
If you have resource limitations on your PC, ensure to deactivate the OpenCV image visualization by setting 
`export_show_image` to `False` in both launch files.

**Parameters**
```
<param name="export_directory" type="str" value="/home/student/CarND-Capstone/export"/>
<param name="export_filename" type="str" value="tfl_"/>
<param name="export_rate" type="int" value="1"/>
<param name="export_encoding" type="str" value="bgr8"/>
<param name="export_show_image" type="bool" value="True"/>
```

**Simulator**
1. Check if the export directory (`export_directory`) exists and is empty. The exporter overrides existing images!
2. Start the image extractor node with styx support by `roslaunch launch/styx_image_extractor.launch`
3. Run the simulator
4. Activate camera output in simulator

**ROS Bags**
1. Check if the export directory (`export_directory`) exists and is empty. The exporter overrides existing images!
2. Start the image extractor node by `roslaunch launch/site_image_extractor.launch`
3. Run ROS bag player by `rosbag play ./bags/just_traffic_light.bag`

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
