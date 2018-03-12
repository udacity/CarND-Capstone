This is the documentation for the deep learning models for the traffic light detection

### Overview
We are making use of the [Tensorflow Object Detection API](https://github.com/tensorflow/models) and have chosen the following pre-trained model:

[faster_rcnn_resnet101_coco_2018_01_28](http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz)

We have then used transfer learning to learn the new objects - specifically, traffic light colours.

### Simulator model -> tld_simulator_model

The current directory contains:

- frozen model
- pipeline configuration
- label map 
- jupyter notebook with examples of usage

The `tld_test_images` directory contains:

- 3 sample images for testing the model via the jupyter notebook

The full data set of annotated simulator images (train/test split) can be downloaded [here](https://drive.google.com/open?id=146sr5zUg1ojYFWN0SN7_TJ41g7Jy7I9c)

### Real world model
This is in development
