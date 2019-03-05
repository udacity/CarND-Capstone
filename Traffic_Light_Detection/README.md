# Traffic Light Detection model training

To drive the car we need to be able to detect and classify traffic lights. To do so, we trained an object detector using TensorFlow.
The final model performs in a single stage both the traffic light detection (and draws a bounding box around it) and the light color classification (between 3 classes: Red, Yellow, Green).

## Preparing the training data
We will train using the following datasets
- [carla_training](https://drive.google.com/open?id=1pk7RZYcP57dxs-xmAHcdHseZ__ZtZWgB): Combination of two datasets of images collected from Carla's testing site. There's a total of 1773 images in jpg format and their corresponding bounding box annotation file in xml with labels (Green, Red, Yellow).
- [carla_testing](https://drive.google.com/open?id=1A_lA1zdfRVDcaht2Z3z_iLkZHs7-QbbM): Dataset of images collected from Carla's testing site, recorded during a sunny day and with some images from that car that have a lot of glare and reflection. There's a total of 329 images. 
- [simulator](https://drive.google.com/open?id=1-G066U5BUUNPvdinWrALX5XWcw7TioeY): Images collected from Udacity's simulator. There's a total of 511 images, in the same format as previous datasets.
- [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132)
- [LISA Traffic Light Dataset](https://www.kaggle.com/mbornoe/visualization-of-lisa-traffic-light-dataset)

To start with clone these 2 repos and download the dataset. if you place the folders like this you should be able to just run the commands as is.

    .
    ├── CarND-Capstone          # This repo
    ├── models                  # tensorflow models repo https://github.com/tensorflow/models.git
    └── dataset                 # dataset
        ├── Bosch               # Bosch dataset
        |   ├── test            
        |   ├── train
        |   ├── test.yaml
        |   └── train.yaml
        └── LISA                # LISA dataset
        |   ├── Annotations            
        |   ├── daySequence1
        |   ├── daySequence2
        |   └── dayTrain
        └── Udacity             # Udacity dataset
            ├── labels            
            ├── sim_y_0110.jpg
            ├── ...jpg

Not all dataset need to be used, and it is possible to just run the following commands for the datasets you wish to use.

### Udacity
In the first iteration of this project, we ended up combining the three first datasets: carla_training, carla_testing, simulator. The combination of these three datasets adds up to a total of 2,613 images. We called this collection **Mixed** dataset. The results of the model sufficiently met our needs, achieving a good balance between accuracy and fast running time. Because of this, we didn't see a priority investing extra time and computation in retraining the model with the other public datasets, given the time constraints we had in delivering the first version of this project. We will consider retraining the model on these datasets on the second iteration of this project.   

In order to train the model using the TensorFlow Object Detection API, the images supplied needed to be converted into [TensorFlow Record file format](https://www.tensorflow.org/guide/extend/formats). We've supplied the utility file `create_tf_record.py` that converts the annotated images into a TensorFlow Record, optionally splitting the dataset into train and validation.

The code to convert the images to TensorFlow Record file format can be found in this notebook: [Traffic-Light-Detection-TFRecord.ipynb](https://github.com/ilopezfr/Traffic_Light_Detection/notebooks/Traffic-Light-Detection-TFRecord.ipynb) 

In that notebook, you can find all the steps to do the conversion. Essentially, one has to download the datasets with the images and labels, combine them together and then run the `create_tf_record.py` program: 
```console
$ python create_tf_record.py --data_dir=data/simulator \
                           --labels_dir=data/simulator/labels \
                           --labels_map_path=config/labels_map.pbtxt \
                           --output_path=data/simulator/simulator.record
```
This program already splits the images into training and evaluation TF Record files. By default is 75% for training and 25% for evaluation.
The final files structure should look like this:

```Console
mixed
    L labels
         L img_01.xml
         L img_02.xml
         L ...
    L img_01.jpg
    L img_02.jpg
    L ...
    L mixed_train.record
    L mixed_eval.record

```

The converted TensorFlow Record files for the training and evaluation can also be downloaded here:
- [mixed_train.record](https://drive.google.com/open?id=1orq0y-8GtfOWl1tBko03rSZT7b3sVfBf)
- [mixed_eval.record](https://drive.google.com/open?id=18nLlxkdJtwfbOaFvpdLhJXrknfzwNNKw) 

### Bosch
The bosch dataset has annotations in YAML format, and the `create_tf_record_yaml.py` is used.

```console
# copy all the training data to one folder
$ find dataset/Bosch/train/ -type f -print0 | xargs -0 mv -t dataset/Bosch/train
# Create the train and test tfrecord
$ python3 CarND-Capstone/Traffic_Light_Detection/create_tf_record_yaml.py --output_path CarND-Capstone/Traffic_Light_Detection/training/data/train_bosch.record --path_to_yaml dataset/Bosch/train.yaml --path_to_images dataset/Bosch/train
$ python3 CarND-Capstone/Traffic_Light_Detection/create_tf_record_yaml.py --output_path CarND-Capstone/Traffic_Light_Detection/training/data/test_bosch.record --path_to_yaml dataset/Bosch/test.yaml --path_to_images dataset/Bosch/test
```
### LISA
The bosch dataset has annotations in YAML format, and the `create_tf_record_yaml.py` is used.

```console
# repeat for as many dayClip's as you want there are 13
$ python3 Traffic_Light_Detection/generate_tfrecords_csv.py 
        --output_path CarND-Capstone/Traffic_Light_Detection/training/data/train_lisa1.record 
        --csv_input dataset/LISA/Annotations/dayTrain/dayClip1/frameAnnotationsBOX.csv 
        --path_to_images dataset/LISA/dayTrain/dayClip1/frames/
$ python3 Traffic_Light_Detection/generate_tfrecords_csv.py 
        --output_path CarND-Capstone/Traffic_Light_Detection/training/data/test_lisa1.record 
        --csv_input dataset/LISA/Annotations/dayTrain/daySequence1/frameAnnotationsBOX.csv 
        --path_to_images dataset/LISA/dayTrain/daySequence1/frames/
$ python3 Traffic_Light_Detection/generate_tfrecords_csv.py 
        --output_path CarND-Capstone/Traffic_Light_Detection/training/data/test_lisa2.record 
        --csv_input dataset/LISA/Annotations/dayTrain/daySequence2/frameAnnotationsBOX.csv 
        --path_to_images dataset/LISA/dayTrain/daySequence2/frames/
```

## Training 

### Model
As Carla the self driving car is installed with Tensorflow 1.3.0 we need to train the model using that version of Tensorflow, and therefore we need this older pretrained model to do the transfer learning from [ssd_mobilenet_v1_coco_11_06_2017](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz)
place the model in CarND-Capstone/Traffic_Light_Detection/models/

### Configuration File
The config file is dependent of the model and datasets used. We adjusted our config file to **sd_mobilenet_v1_coco model** and the mixed dataset of 1833 images.

We followed the instructions for [Configuring Object Detection Training Pipeline](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/configuring_jobs.md). 

Taking the template config file provided by Tensorflow, we simply made the following adjustments:
- Added the input_path to the train and validation tfrecord files.
- Edited the label_map_path to config/labels_map.pbtxt
- Updated num_classes: 3 (for this model we have 3 classes only: red, green, yellow)
- The num_examples in the evaluation section that correspond to the number of samples in the evaluation record
- The ssd_anchor_generator section, updating the scales and removing unused aspect ratios (the traffic lights are ~ 0.33)
- Reduced the number of detections from 100 to 10 in max_detections_per_class and max_total_detections

### Training
#### Training locally
Before we can start training using Tensorflow object detection we need to setup our environment following the [Installation instructions]
It is important to be in the /models/research dir when running 
`export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim` as we have not copied slim to CarND-Capstone

The model can now be trained
```console
$ cd CarND-Capstone/Traffic_Light_Detection/training
$ python3 train.py --logtostderr --train_dir=models/train --pipeline_config_path=ssd_mobilenet_v1_coco.config
```
#### Training Colab
We trained the model using Colab in a GPU for 20,000 global steps on the Mixed Dataset. The whole training took approximately 9 hours. The list of parameters used: 
- batch_size = 24
- Steps = 20000
- Learning_rate = 0.004
- Anchors min scale = 0.1
- Anchors max scale = 0.5
- Anchors Aspect Ratio = 0.33

We've provided the notebook used in Colab for the full training: [Traffic-Light-Detection-Training.ipynb](https://github.com/ilopezfr/Traffic_Light_Detection/notebooks/Traffic-Light-Detection-Training.ipynb)

## Export the model

In order to use the model for inference in production the graph must be freezed. TensorFlow provides an utility to export the frozen model: [Exporting Trained Model](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/exporting_models.md). 

```Console
$ python object_detection/export_inference_graph.py \
        --input_type=image_tensor \
        --pipeline_config_path=config/ssd_mobilenet_v1.config \
        --trained_checkpoint_prefix=models/fine_tuned/ssd_mobilenet/model.ckpt-20000 \
        --output_directory=models/exported/ssd_mobilenet
```

## Conversion to Tensorflow v1.3
We learned that the software stack included in Carla (Udacity's self-driving car) uses Tensorflow's version 1.3. 

Unfortunately, the model that we originally trained in Colab was using using a newer version of Tensorflow. Moreover, TF Object Detection API only goes back to Tensorflow 1.4 (which can be found in one this [commit](https://github.com/tensorflow/models/commit/edcd29f2dbb4b3eaed387fe17cb5270f867aec42).

Luckily it appears that models converted to TF 1.4 are also compatible with TF 1.3. To convert the model we followed this procedure: 
```Console
# Create conda env for tensorflow 1.4
$ conda create -n tensorflow_1.4 python=3.6
$ conda activate tensorflow_1.4
# Install Tensorflow 1.4.0
$ conda install tensorflow==1.4.0
# Install dependencies
$ conda install pillow lxml matplotlib
# Clone the tensorflow object models repo and checkout compatible version
$ git clone https://github.com/tensorflow/models.git temp
$ cd temp
$ git checkout d135ed9c04bc9c60ea58f493559e60bc7673beb7
# Copy temp/research/object_detection and temp/research/slim to the exporter
$ mkdir exporter
$ cp -r temp/research/object_detection exporter/object_detection
$ cp -r temp/research/slim exporter/slim
$ cd exporter
# Download [protoc 3.4.0](https://repo1.maven.org/maven2/com/google/protobuf/protoc/3.4.0/)and extract the protoc.exe into /exporter
# Compile proto buffers:
$ protoc.exe object_detection/protos/*.proto --python_out=.
# Set PythonPath:
$ SET PYTHONPATH=%cd%;%cd%\slim
# Run test
$ python object_detection/builders/model_builder_test.py
# Export the model
$ python object_detection/export_inference_graph.py --input_type=image_tensor --pipeline_config_path=../config/config/ssd_mobilenet_v1.config --trained_checkpoint_prefix=../models/fine_tuned/ssd_mobilenet/model.ckpt-20000 --output_directory=../models/converted_to_TFv14/ssd_mobilenet
```

## Evaluation

To evaluate the model, we passed a set of sample images selected from both the simulator and the real test site images from Carla. 

Overall, the model performs well although it could further be optimized. Looking at the false positives obtained, we could appreciate how the model sometimes misclassifies the color in saturated light condition (with glare and reflection). In the future, we will explore implementing Gamma Correction technique to alleviate this issue. 

The notebook used for the evaluation can be found here: [Traffic-Light-Classifier-Evaluation](https://github.com/ilopezfr/Traffic_Light_Detection/notebooks/Traffic-Light-Detection-evaluation.ipynb) 

![alt text][image2]   |![alt text][image3]
:--------------------:|:--------------------:
![alt text][image4]   |![alt text][image5]
![alt text][image6]   |![alt text][image7]
![alt text][image8]   |![alt text][image9]
![alt text][image10]  |![alt text][image11]
![alt text][image12]  |![alt text][image13]

# Deployment

Finally, we can copy the graph to the *tl_detector* node so that our car can use it when running. 

```console
$ cp models/converted_to_TFv14/ssd_mobilenet/frozen_inference_graph.pb ../../ros/src/tl_detector/light_classification/model/
```

## Thanks
Thanks to Daniel Stang for the nice [Object Detection API Tutorial](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e), Anthony Sarkis for his [TFRecord script](https://github.com/swirlingsand/deeper-traffic-lights/blob/master/data_conversion_bosch.py), Marco Marasca for his detailed guide on the [model training](https://github.com/Az4z3l/CarND-Traffic-Light-Detection) and conversion to TF 1.3 and for providing the link to the aggregated mixed datasets. 

## Acknowledgements
Thanks for creating the LISA Traffic Light Dataset :-)

Jensen MB, Philipsen MP, Møgelmose A, Moeslund TB, Trivedi [MM. Vision for Looking at Traffic Lights: Issues, Survey, and Perspectives](https://ieeexplore.ieee.org/document/7398055/). I E E E Transactions on Intelligent Transportation Systems. 2016 Feb 3;17(7):1800-1815. Available from, DOI: 10.1109/TITS.2015.2509509

Philipsen, M. P., Jensen, M. B., Møgelmose, A., Moeslund, T. B., & Trivedi, M. M. (2015, September). [Traffic light detection: A learning algorithm and evaluations on challenging dataset](https://ieeexplore.ieee.org/document/7313470). In intelligent transportation systems (ITSC), 2015 IEEE 18th international conference on (pp. 2341-2345). IEEE.



[//]: # (Image References)

[image1]: ./test_images/sim_r.jpg "Sim R"
[image2]: ./test_images_result/sim_r_result.png "Sim R result"
[image3]: ./test_images_result/sim_u_result.png  "Sim U result"
[image4]: ./test_images_result/sim_y_result.png "sample 4"
[image5]: ./test_images_result/uda_lights_g_result.png "sample-5"
[image6]: ./test_images_result/uda_lights_y_result.png "sample 6"
[image7]: ./test_images_result/uda_loop_g_result.png "sample 7"
[image8]: ./test_images_result/uda_loop_r_result.png "sample 8"
[image9]: ./test_images_result/uda_loop_u_2_result.png "sample 9"
[image10]: ./test_images_result/uda_loop_u_result.png  "sample 10"
[image11]: ./test_images_result/uda_training_g_result.png "sample 11"
[image12]: ./test_images_result/uda_training_r_result.png  "sample-12"
[image13]: ./test_images_result/uda_training_y_result.png  "sample-13"