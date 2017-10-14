# How to check your model with a recorded rosbag
1. filter the original rosbag file to exclude the topic to check, i.e., `/traffic_waypoint` in this case.
  - e.g., `rosbag filter original.bag check.bag "topic!='/traffic_waypoint'"`
2. replay the check bag to create the environment
  - e.g., `rosbag play -l check.bag`
3. run your model at the same time
  - e.g., `roslaunch launch/site.launch`
4. monitor the model behavior by checking the relevant topic
  - e.g., `rostopic echo /traffic_waypoint`
5. to visually check the result, you may also want to make a video based on image topic, e.g., `/raw_image` by
  - extract images
  - make an animation based on the right frequency.