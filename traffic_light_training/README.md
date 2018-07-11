# Capstone Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./training_trafficlight_sg/training_monitoring.png "moni1"
[image2]: ./training_trafficlight_sg/training_monitoring2.png "moni2"

[image3]: ./training_trafficlight_sg/detection1.jpg "detect1"
[image4]: ./training_trafficlight_sg/detection1b.jpg "detect2"
[image5]: ./training_trafficlight_sg/detection1c.jpg "detect3"
[image6]: ./training_trafficlight_sg/detection2.jpg "detect4"
[image7]: ./training_trafficlight_sg/detection2b.jpg "detect5"
[image8]: ./training_trafficlight_sg/detection3.jpg "detect6"

[image9]: ./training_trafficlight_sg/daySequence1.jpg "orig_modality1"
[image10]: ./training_trafficlight_sg/daySequence1b.jpg "orig_modality2"

[image11]: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/img/kites_detections_output.jpg "tensorflow_object"


## Training a Traffic Lights Detection and Classification Network

This task is about the perception module of the capstone project of the Udacity Self-Driving Car Engineer Nanodegree Program.

To build up a robust and computational efficient system for traffic light detection and classification based on a deep neural network architecture, one might think about the following points:

- [What framework to use for training and inference (TensorFlow can be used in the car environment)](#framework)
- [What neural network architecture to use](#neural-network-architectures)
- [What datasets for training can be used](#dataset)
- How to augment images for creating a robust network
- How to monitor the training of the chosen model
- How to deploy the trained model to the car environment


### Framework

The [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) is a way to tackle the problem. Next to other really handy frameworks like [Darknet YOLO](https://pjreddie.com/darknet/yolo/), it is easy to use for fast prototyping, training, training monitoring and graph exports.

![detectionAPI][image11]

### Neural Network Architectures

There are several architectures for object detection that have pros and cons. To name some of them:

- Faster RCNN Architectures with different backends (slow but very accurate)
- SingleShotDetectors (SSD) with different backends (very fast but not as accurate)
- You Only Look Once (YOLO) with Darknet backends (fast and accurate)


### Pre-Trained Network

It is usefull to start with a pre-trained network and fine tune it on the specific task. Most networks are pre-trained on object-detection datasets like [COCO Dataset](http://cocodataset.org/#home) 


### Dataset

For this project, I want to use the following datasets for fine tuning:
- [Lisa Traffic Light Dataset](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/home)
- ~~[CamVid](http://mi.eng.cam.ac.uk/research/projects/VideoRec/CamVid/)~~ (in the end not used)
- [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132)
- [Udacity Simulator and Site Dataset packed into TFRecord files by Shyam Jaganathan](https://drive.google.com/drive/folders/0Bz-TOGv42ojzOHhpaXJFdjdfZTA)

#### How to prepare the dataset

This section describes how to prepare the different datasets for unified training. 

* Download the dataset files
* Extract the zip files and arrange the data in the following way:

In my case, the base directory is **/data/Datasets**

```
. \
├── _LisaTrafficLight
    ├── _Annotations
        ├── dayTrain
        └── dayVal
    ├── _dayTraining
        ├── dayClip1
        └── ...
    ├── _dayTest
        ├── daySequence1
        └── daySequence2
        
├── Bosch Traffic Lights Dataset
    ├── train.yaml
    ├── test.yaml
    └── _rgb
        └── _test
            ├── 24968.png
            └── ...
        └── _train       
            ├── 2015-10-05-10-52-01_bag
            └── ...
            
├── _UdacityData
    ├── traffic-light-site-train.record
    ├── traffic-light-sim-train.record
    ├── traffic-light-site-test.record
    └── traffic-light-sim-test.record
```

After that, have a look into the main-function of the [conversion script](scripts/utilities/convert_datasets/conversion.py). From line 351 starting, the main function executes the

* Annotation file parsing for each dataset
* Conversion of the annotation to a unified python dictionary
* Dataset image frequency normalization
* Conversion to TFRecords (for training and validation)

Take care of the base path which is **/data/Datasets** by default in the script.

#### Dataset normalization

Each dataset consists of a different number of samples. To harmonize training and not fit the network to one of the given image modalities, the dataset image countings are being normalized in the conversion.py script. This is done by random sampling until the needed number of samples is reached. These are the counts of the specific datasets before the normalization:

|Dataset name|Number of train images|Number of val images|
|-|-|-|
|Lisa Traffic Light Dataset|12775|7473|
|Bosch Small Traffic Lights Dataset|3153|7147|
|Udacity Simulator Images|494|871|
|Udacity Site Images|871|41|

And after normalization:

|Dataset name|Number of train images|Number of val images|
|-|-|-|
|Lisa Traffic Light Dataset|12775|7473|
|Bosch Small Traffic Lights Dataset|12775|7473|
|Udacity Simulator Images|12775|7473|
|Udacity Site Images|12775|7473|


### Image Augmentation

To train a robust object detector with built-in classifier, image augmentation is a crucial point. This regularization method enforces the network to better generalize to unseen images, and thus be invariant to different image scences.

The Tensorflow Object Detection API offers several image augmentation methods, that can be applied during training. For this task, I enabled the following augmentations:

|Method|Description/Reason|
|-|-|
|random_horizontal_flip|To generate 'more' data out of the given data. Since the traffic light scenes are invariant to horizontal flip, this generates new 'mirrored' image data.|
|random_pixel_value_scale|A good technique to change the pixel value in a random manner (for all pixels in the image).|
|random_adjust_brightness|Change the brightness of the image (like pixel value scale) by a random factor.|
|random_adjust_contrast|Change the contrast of the image by a random factor.|
|random_rgb_to_gray|Randomly changes the RGB colorspace to grayscale. This should force the network to extract the location features of the traffic light lamp.|
|random_adjust_hue|Random change of hue. This may force the network to rely on the traffic light lamp position, not as much on the color feature (since the color of the red lamp in the site images looks rather orange!)|
|random_distort_color|This performs a cascade of different color distortion functions.|
|random_jitter_boxes|Randomly jitters boxes.|
|ssd_random_crop|Randomly crops the image.|

### Training

Before the training begins, the specific datasets have to be converted to the framework-specific format. In this case to TF Record files. Three classes (red, yellow, green) are encoded with their bounding boxes.

The actual training is done via [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) and is monitored with Tensorboard.

![monitor1][image1]

Both, precision values and validation images can be shown in Tensorboard.

![monitor2][image2]




## Iteration 1 - Testing Faster RCNN (ResNet50) on Lisa Traffic Light Dataset

Iteration 1 is done with finetuning a *Faster RCNN Network with ResNet50 Backbone (pretrained on MS COCO)*. This network is fine tuned 200k steps on the [Lisa Traffic Light Dataset](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/home).

The images of the Lisa Traffic Light Dataset look like this:

| Day Scene 1 | Day Scene 2 |
:-:|:-:
![orig_1][image9]|![orig_2][image10]



### Current results on Udacity-images
---

| | |
:-:|:-:
![detection1][image3]|![detection2][image4]
![detection3][image5]|![detection4][image6]
![detection5][image7]|![detection6][image8]


## Work to do

- Training on the other datasets
- Speed up by training different architectures
- Get invariance of traffic light size
- Integrate into main project



