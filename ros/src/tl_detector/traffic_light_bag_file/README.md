## Export the image data from the rosbag of carla

To collect the data from rosbag, we use the image_view package.

The rosbag file need to be in this dir:
* traffic_light_training.bag (https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip)

(Optional) you can play the rosbag in rviz by using the config file:
* default.rviz (https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/default.rviz)

### Usage:
```
roslaunch /devdisk/Udacity-Project/CarND-Capstone-1/ros/src/tl_detector/traffic_light_bag_file/export_img_from_rosbag.launch
mkdir ../carla_data
mv ~/.ros/frame*.jpg ../carla_data/
```

