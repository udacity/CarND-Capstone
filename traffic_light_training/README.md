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
[image12]: ./training_trafficlight_sg/datasets_images.png "datasets"
[image13]: ./training_trafficlight_sg/training_monitoring3.png "moni3"

## Training a Traffic Lights Detection and Classification Network

This task is about the perception module of the capstone project of the Udacity Self-Driving Car Engineer Nanodegree Program.

To build up a robust and computational efficient system for traffic light detection and classification based on a deep neural network architecture, one might think about the following points:

- [What framework to use for training and inference (TensorFlow can be used in the car environment)](#framework)
- [What neural network architecture to use](#neural-network-architectures)
- [What datasets for training can be used](#dataset)
- [How to setup training with the chosen framework](#setup-the-training-and-evaluation-pipeline)
- [How to augment images for creating a robust network](#image-augmentation)
- [How to monitor the training](#training-monitoring)
- [Train a model for proof of concept](#proof-of-concept-testing-faster-rcnn-resnet50-on-lisa-traffic-light-dataset)
- [Train the actual model](#training-the-single-shot-detector-with-mobilenet-backend)
- [How to deploy the trained model to the car environment](#deploy-to-the-ros-system)


### Framework
-----
The [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) is a way to tackle the problem. Next to other really handy frameworks like [Darknet YOLO](https://pjreddie.com/darknet/yolo/), it is easy to use for fast prototyping, training, training monitoring and graph exports.

![detectionAPI][image11]

### Neural Network Architectures
-----
There are several architectures for object detection that have pros and cons. To name some of them:

- Faster RCNN Architectures with different backends (slow but very accurate)
- SingleShotDetectors (SSD) with different backends (very fast but not as accurate)
- You Only Look Once (YOLO) with Darknet backends (fast and accurate)


#### Pre-Trained Network

It is usefull to start with a pre-trained network and fine tune it on the specific task. Most networks are pre-trained on object-detection datasets like [COCO Dataset](http://cocodataset.org/#home) 


### Dataset
-----
For this project, I want to use the following datasets for fine tuning:
- [Lisa Traffic Light Dataset](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/home)
- ~~[CamVid](http://mi.eng.cam.ac.uk/research/projects/VideoRec/CamVid/)~~ (in the end not used)
- [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132)
- [Udacity Simulator and Site Dataset packed into TFRecord files by Shyam Jaganathan](https://drive.google.com/drive/folders/0Bz-TOGv42ojzOHhpaXJFdjdfZTA)

![datasetsimages][image12]


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


**The script outputs train_records.tfrecord and val_records.tfrecord.**


### Setup the Training and Evaluation Pipeline
-----

To use the [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) for the Udacity Capstone project, it is nessecary to clone the [Github Repository tensorflow/models](https://github.com/tensorflow/models). 

But since the car *Carla* only supports Tensorflow 1.3.0, its crucial to checkout an older release of the repo. The official release tag 1.4.0 would be ok, but they removed the research models that we need. 

* So I used this [commit as my code basis for the Tensorflow Object Detection API](https://github.com/tensorflow/models/commit/d1cdc444efb93bb4c38c6fbc619ece0b3ae3582a).


#### Choose the pretrained model

Having a look at the [Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md), the **ssd_mobilenet_v1_coco** is the only one that is really fast and supported by Tensorflow 1.3.0. So this may be the right choice. 

* Download the pretrained weights for this model.


#### Create the training configuration

There are really good example configuration files in the [tensorflow respository](https://github.com/tensorflow/models/tree/master/research/object_detection/samples/configs). The resulting script for starting the training can be found in scripts: [ssd_mobilenetv1_coco_pipeline.config](scripts/object_detect_training/ssd_mobilenetv1_coco_pipeline.config).

Here are the main parts I have changed in this configuration file:
* num_classes
* resizer to fixed_shape_resizer 
* learning rate to manual_step_learning_rate 
* data_augmentation_options
* fine_tune_checkpoint 
* train_input_reader
* eval_input_reader
* eval_config


#### Start the training and monitoring

To start the training and monitoring, adjust the paths in the [run_training_SSD_MobileNetV1.sh](scripts/object_detect_training/run_training_SSD_MobileNetV1.sh) and [run_eval_SSD_MobilenetV1.sh](scripts/object_detect_training/run_eval_SSD_MobilenetV1.sh) and start Tensorboard in the respective output directory! Be sure the traindata is located properly in the *traindata* and *valdata* directories.


#### Evaluate after training

To test the model after training, adjust the paths in the [export_graph_SSD_MobileNet.sh](scripts/object_detect_training/export_graph_SSD_MobileNet.sh) script and run it to freeze and export the tensorflow graph with embedded weights.

After that, you can use the [inference.py](scripts/object_detection_inference/inference.py) script to test the model and inpaint the detections. Be sure to configure your project correctly, so that the object detection code can be included properly.



### Image Augmentation
-----
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


### Training monitoring
-----
The Training is monitored with Tensorboard. To generate the events for tensorboard, the [run_eval_SSD_MobilenetV1.sh](scripts/object_detect_training/run_eval_SSD_MobilenetV1.sh) script must be executed in parallel (!) to the training process.

![monitor1][image1]

Both, precision values and validation images can be shown in Tensorboard.

![monitor2][image2]


### Proof of concept: Testing Faster RCNN (ResNet50) on Lisa Traffic Light Dataset
-----
The first test is done with finetuning a *Faster RCNN Network with ResNet50 Backbone (pretrained on MS COCO)*. This network is fine tuned 200k steps and only on the [Lisa Traffic Light Dataset](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/home).

Results with the Faster RCNN ResNet architecture:

| | |
:-:|:-:
![detection3][image5]|![detection4][image6]
![detection5][image7]|![detection6][image8]

The network was able to generalize to the Udacity site images.


### Training the Single Shot Detector with MobileNet Backend
-----
After the proof of concept succeeded, the actual training of the SSD MobileNet can be done. This training is now with the help of all four datasets, encoded in the TFRecord files.

The training process runs for around 80.000 steps and reaches a mean Average Precision (at 0.5 Intersection over Union):

|Class|mAP @0.5 IOU|
|-|-|
|Green|0.52|
|Yellow|0.86|
|Red|0.68|
|N/A|0.93|

The network is able to generalize over all four modalities.

![tb3][image13]



### Deploy to the ROS system
-----

To deploy the model to the ROS system, the frozen graph has to be used. The tensorflow graph is imported via *tf.import_graph_def()* in the [ROS node for Traffic Light Classification](../ros/src/tl_detector/light_classification/tl_classifier.py). The most important thing to think of are the tensors we need from the graph. These are the graph tensors that are being used for inference:

|Tensor name|Description|
|-|-|
|image_tensor:0|The tensor where the input image is fed into|
|num_detections:0|The count of the detections. This is static and depends on the training configuration.|
|detection_boxes:0|The respective bounding boxes that refer to the detections.|
|detection_scores:0|The respective scores or probabilities for the detections.|
|detection_classes:0|The respective class for the detections.|

We filter the scores with a threshold of 0.5.


To test the model, the class TestTLClassifier in tl_classifier.py is used. The detection runs on a virtual machine on CPU with about **8 ms per image**. The images are subsampled to 400x300 pixels before they are fed into the model.

