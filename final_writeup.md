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
source: ros/src/tl_detector/
|subscribed to|publish to|
|/image_color||
|/current_pose|
|/base_waypoints|


## control
## waypoint following
