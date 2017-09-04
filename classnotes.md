# Notes taken from Udacity class

## Nodes

### 1. traffic light detector
- subscribed topics:
	- `/base_waypoints`: repeated list of all waypoints
	- `/camera/image_raw`: updated images
	- `/current_pose`: current position of vehicle
- published topics:
	- `/traffic_waypoint`: 
- helper topics:
	- `/vehicle/traffic_lights`: simulator publishes the location and current color state of all traffic lights in the simulator. It can be used for training data collection or as a stub for testing of other components.
- node files:
	- `tl_detector/tl_detector.py`: node file
	- `tl_detector/light_classification_model/tl_classifier.py`: classifier model
	- `tl_detector/traffic_light_config`: This config file contains information about the camera (such as focal length) and the 3D position of the trafic lights in world coodinates

### 2. waypoint updater

The purpose of the node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.

- subscribed topics:
	- `/base_waypoints`: repeated list of all points
	- `/obstacle_waypoints`: NOT THERE!
	- `/traffic_waypoint`
	- `/current_pose`
- published topics:
	- `/final_waypoints`: only waypoints ahead up to `LOOKAHEAD_WPS`.
- node files:
	- `waypoint_updater/waypoint_updater.py`

### 3. twist controller (DBW drive-by-wire)
- subscribed topics:
	- `/current_velocity`
	- `/twist_cmd`
	- `/vehicle/dbw_enabled`
- published topics:
	- `/vehicle/throttle_cmd`
	- `/vehicle/steering_cmd`
	- `/vehicle/brake_cmd`
- node files:
	- `twist_controller/dbw_node.py`

## Message types

Intuition behind sevearl types
- a `lane` is a collection of waypoints with header
- a `waypoint` consists of `pose` and `twist`
- `pose` has a xyz `position` and xyzw `orientation`
- `twist` has a `linear` (xyz) speed and `angular` (xyz) speed

In details

- `/base_waypoints`: styx_msgs/Lane
- `/camera/image_raw`: sensor_msgs/Image
- `/current_pose`: geometry_msgs/PoseStamped
- `/final_waypoints`: styx_msgs/Lane
- `/traffic_waypoint`: std_msgs/Int32 (indx of all /base_waypoints)
- `/current_velocity`: geometry_msgs/TwistStamped
- `/twist_cmd`: geometry_msgs/TwistStamped
- `/vehicle/dbw_enabled`: std_msgs/Bool
- `/vehicle/throttle_cmd`: dbw_mkz_msgs/ThrottleCmd
- `/vehicle/steering_cmd`: dbw_mkz_msgs/SteeringCmd
- `/vehicle/brake_cmd`: dbw_mkz_msgs/BrakeCmd
