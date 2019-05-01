# simulator has two test tracks:

A highway test track with traffic lights
  the first track has a toggle button for camera data. Many students have experienced latency when running the simulator together with a virtual machine, and leaving the camera data off as you develop the car's controllers will help with this issue.
A testing lot test track similar to where we will run Carla
  To use the second test lot, you will need to update your code to specify a new set of waypoints. We'll discuss how to do this in a later lesson.

# units
the simulator displays vehicle velocity in units of mph. However, all values used within the project code are use the metric system (m or m/s), including current velocity data coming from the simulator.

# team size
Note that we'd like a minimum team size of 4
# components
## traffic light detection
* package source: ros/src/tl_detector/
* /current_pose topic : provides the vehicle's current position
* /base_waypoints topic: provides a complete list of waypoints the car will be following.
* tl_classfier.py: tl_detector/light_classification_model/tl_classfier.py
*  tl_detector.py : tl_detector/tl_detector.py

|   node     | subscribed to | publish to |
| :-----------: |:-------------:| :-----:|
| tl_detector.py | /image_color | /traffic_waypoint |
| | /current_pose | |
| | /base_waypoints | |
| tl_classfier.py | |  |
| waypoint_updater.py | /current_pose | /final_waypoints |
| | /base_waypoints | |
| | /obstacle_waypoint | |
| | /traffic_waypoint | |


## control
## waypoint following
* package source: ros/src/waypoint_updater/
* purpose: to update the target velocity property of each waypoint based on traffic light and obstacle detection data
