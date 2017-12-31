# Traffic Light Detection and Classification Model

[//]: # (Image References)
[image_generator_label_heatmap_all]: ./images/generator_label_heatmap_all.png
[image_generator_label_heatmap_rygo]: ./images/generator_label_heatmap_red_yellow_green_off.png
[image_generator_output]: ./images/generator_output.png

[image_bosch_labeled_image]: ./images/bosch_labeled_image_0.png
[image_bosch_label_hist]: ./images/bosch_label_histogram.png
[image_bosch_label_heatmap_all]: ./images/bosch_label_heatmap_all.png
[image_bosch_label_heatmap_rygo]: ./images/bosch_label_heatmap_red_yellow_green_off.png

[image_lara_labeled_image_0]: ./images/lara_labeled_image_0.jpg
[image_lara_labeled_image_1]: ./images/lara_labeled_image_1.jpg
[image_lara_label_hist]: ./images/lara_label_histogram.png
[image_lara_label_heatmap_all]: ./images/lara_label_heatmap_all.png
[image_lara_label_heatmap_rygo]: ./images/lara_label_heatmap_red_yellow_green_off.png

[image_capstone_labeled_image]: ./images/capstone_labeled_image_0.jpg
[image_capstone_label_hist]: ./images/capstone_label_histogram.png
[image_capstone_label_heatmap_all]: ./images/capstone_label_heatmap_all.png
[image_capstone_label_heatmap_rygo]: ./images/capstone_label_heatmap_red_yellow_green_off.png

[image_capstone_sim_labeled_image]: ./images/capstone_sim_labeled_image_0.jpg
[image_capstone_sim_label_hist]: ./images/capstone_sim_label_histogram.png
[image_capstone_sim_label_heatmap_all]: ./images/capstone_sim_label_heatmap_all.png
[image_capstone_sim_label_heatmap_rygo]: ./images/capstone_sim_label_heatmap_red_yellow_green_off.png

## Environment Setup

### Conda Environment
The conda environment `carnd-term3` currently just includes `tensorflow==1.3` because not every team member has access to a NVIDA graphic card. For model training with GPU support change `tensorflow==1.3` to `tensorflow-gpu==1.3` in the [environment.yml](environment.yml) file and update your conda environment.

Setup the conda environment `carnd-term3`:
```
conda env create -f environment.yml
```

Update the conda environment `carnd-term3`:
```
conda env update -f environment.yml
```
### Installation of the Tensorflow Object Detection API

The conda environment [environment.yml](environment.yml) already includes all required packages and library. So just start with the Protobuf compilation and path setups.

https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md

### Required Directory Layout
```
|-- tl_model
    |-- datasets
    |-- dataset_bosch_small_tlr
    |   |-- dataset_test_rgb
    |   `-- dataset_train_rgb
    |-- dataset_lara
    |   `-- Lara3D_UrbanSeq1_JPG
    |-- dataset_sdcnd_capstone
    |   |-- real_training_data
    |   `-- sim_training_data
    `-- model
        `-- research
            |-- object_detection
            |   |-- ...
            |   |-- test_images
            |   |-- tl_model_config
            |   |   `-- rfcn_resnet101_coco_2017_11_08
            |   |-- tl_model_eval
            |   |-- tl_model_freeze
            |   |-- tl_model_test_results
            |   `-- tl_model_training
            `-- slim
```

## Traffic Light R-FCN Model

Background information, performance and runtime analysis results can be read in this paper
[R-FCN: Object Detection via Region-based Fully Convolutional Networks, Jifeng Dai Yi (Microsoft Research), Li∗ Kaiming (Tsinghua University), He Jian Sun (Microsoft Research)](https://arxiv.org/pdf/1605.06409.pdf).

### How to train the model

1. Ensure the conda environment `carnd-term3` is installed and activated
   - ***The current Tensorflow Object-Detection API requires Tensorflow v1.4. For the final submission we have to find an earlier version which supports v1.3.***
2. Download the datasets and setup the directory layout as described above.
3. Download the pre-trained model from the [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
   - Choose the [rfcn_resnet101_coco_2017_11_08](http://download.tensorflow.org/models/object_detection/rfcn_resnet101_coco_2017_11_08.tar.gz) model and unzip it into `CarND-Capstone/tl_model/model/research/object_detection/tl_model_config`.
4. Prepare Tensorflow object detection API according these [instructions](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md)
   - All required packages are already installed in the conda environment `carnd-term3`
   - Each time you open a new terminal, ensure you've updated the python path in the `model/research/` directory.
    ```
    export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
    ```
5. Convert the dataset to TFRecord format
   - Currently the Bosch Small Traffic light and the Capstone (sim+real) datasets are converted. The LARA dataset consists of several ambiguous labels and thus is ignored for the first test runs.
   - The following command splits the dataset into a 85% training and 15% test set without image augmentation.
   ```
   python DatasetToTFRecordConverter.py \
          --train_output_file datasets/train_bosch_capstone.record \
          --test_output_file datasets/test_bosch_capstone.record \
          --train_ratio 0.85 \
          --augmentation_rate 0.0
   ```
   - The following command splits the dataset into a 85% training and 15% test set with 65% image augmentation and total number of images (train + test set) of 15.000 images.
   ```
   python DatasetToTFRecordConverter.py \
          --train_output_file datasets/train_bosch_capstone_augmented.record \
          --test_output_file datasets/test_bosch_capstone_augmented.record \
          --train_ratio 0.85 \
          --augmentation_rate 0.65 \
          --number_images 15000
   ```
6. Change to the directory to `CarND-Capstone/tl_model/model/research/object_detection`
7. Prepare the model configuration in `CarND-Capstone/tl_model/model/research/object_detection/tl_model_config`
   - The `rfcn_resnet101_coco_traffic_light.config` specifies the whole training process. In the `train_config` chapter you can find some hyperparameters like the `batch_size`, the `learning_rate`and the `num_steps`.
8. Run the model training
    ```
    python train.py \
           --logtostderr \
           --train_dir=tl_model_training/ \
           --pipeline_config_path=tl_model_config/rfcn_resnet101_coco_traffic_light.config
    ```
   - All checkpoints are stored in `CarND-Capstone/tl_model/model/research/object_detection/tl_model_training`
   - If you need the checkpoints, move the content of this directory to a safe place before you run the training.
9. Run the evaluation script in a separate terminal. This script performs an TL inference after each checkpoint and shows the images including bounding boxes in Tensorboard.
    ```
    python eval.py \
           --logtostderr \
           --pipeline_config_path tl_model_config/rfcn_resnet101_coco_traffic_light.config \
           --checkpoint_dir tl_model_training \
           --eval_dir tl_model_eval
    ```
10. Run Tensorboard in order to check the total loss value. In case the total loss, the individual losses of the box classifier (classification and localization) or the RPN (region proposal network) start to fluctuate, adjust the batch size and learning rate in the `rfcn_resnet101_coco_traffic_light.config` file.
    ```
    tensorboard --logdir training:tl_model_training,evaluation:tl_model_eval
    ```
11. Freeze the model after successful training
    ```
    python export_inference_graph.py \
            --input_type image_tensor \
            --pipeline_config_path tl_model_config/rfcn_resnet101_coco_traffic_light.config \
            --trained_checkpoint_prefix tl_model_training/model.ckpt-305 \
            --output_directory tl_model_freeze
    ```
 - Choose the checkpoint you like to freeze by changing the number in `tl_model_training/model.ckpt-305`.
 - The frozen model is stored in `CarND-Capstone/tl_model/model/research/object_detection/tl_model_freeze`
12. Test the trained model in the jupyter notebook
    ```
    jupyter notebook
    ```
    - Open the `object_detection_tl_test.ipynb` notebook and run all steps excepting the `Prepare R-FCN Resnet-101 Coco Model`. The code loads the pre-trained model instead of the traffic light model. You can use this code in order to analyze the performance of the pre-trained model without the specific traffic light color classification.
13. Performance evaluation coming soon....

***Good Luck!***

## DatasetHandler
To get a first impression about the dataset, run the `DatasetHandler.py` with the following arguments. It plays a short video with all labeled traffic lights for the Bosch Small Traffic Light, the LARA and the SDCND Capstone Dataset.

```
python DatasetHandler.py \
  --bosch_label_file datasets/dataset_bosch_small_tlr/dataset_train_rgb/train.yaml \
  --lara_label_file datasets/dataset_lara/Lara_UrbanSeq1_GroundTruth_GT.txt \
  --capstone_label_file datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml \
  --show_images \
  --disable_filter
```

### Integration into TL Model Training SW-Component

The integration of the `DatasetHandler`is straight forward.

1. Initialize the `DatasetHandler` with the desired output image size (width, height)
2. Read all desired datasets. Each `read_all_...()` method call concatenates the dataset to the internal one.
3. Split the internal dataset into a training and a validation dataset.
4. Setup a  training and a validation generator with the desired batch size and augmentation rate.

```python
# initialize the DatasetHandler with the model input layer size (image size)
dataset_handler = DatasetHandler(width=1024, height=768)

# read desired datasets
dataset_handler.read_all_bosch_labels('path to Bosch training yaml file')
dataset_handler.read_all_lara_labels('path to LARA ground truth txt file')
dataset_handler.read_all_capstone_labels('path to capstone real data yaml file')
dataset_handler.read_all_capstone_labels('path to capstone sim data yaml file')

# shuffles and splits the dataset into a 80% training and a 20% test, resp. validation set
train_samples, validation_samples = dataset_handler.split_dataset(train_size=0.8)

# initialize generators for model training and validation with a
# batch size of 128 and 65% augmentation rate
train_generator = dataset_handler.generator(train_samples, batch_size=128, augmentation_rate=0.65)
validation_generator = dataset_handler.generator(validation_samples, batch_size=128, augmentation_rate=0.65)

# start model training...
```

#### Convenient Methods for Dataset Loading
In order to load all available dataset at once use the following method.

***Attention:*** *In order to ensure a correct dataset preparation, setup the directory layout exactly as described in the chapter "Environment Setup" above.*

```python
dataset_handler.read_predefined_dataset()
```

#### Generator Output
The `generator()` outputs a list with sample images (see left image below) and list with the ground truth images (see right image below). The list contains `batch_size` RGB images in the specified size `[width, height, 3]`. Alternatively, all bounding boxes instead of the ground truth image, can be returned as well. The bounding boxes are required to train the network with for the Tensorflow Object-Detection API.

![Generator output][image_generator_output]

The image above can be generated with the following arguments:
```
python DatasetHandler.py \
  --bosch_label_file datasets/dataset_bosch_small_tlr/dataset_train_rgb/train.yaml \
  --lara_label_file datasets/dataset_lara/Lara_UrbanSeq1_GroundTruth_GT.txt \
  --capstone_label_file datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml \
  --show_generator \
  --disable_filter
```

**Ground Truth Color Coding**

| Annotation      | RGB Color-Code |
|:----------------|:--------------:|
| GT_TL_RED       |    0xFF0000    |
| GT_TL_YELLOW    |    0xFFFF00    |
| GT_TL_GREEN     |    0x00FF00    |
| GT_TL_UNDEFINED |    0x646464    |

**Augmentation Results**

In total 65% of the whole dataset has been augmented by the following methods. The augmentation methods are combined with its own probability.

- random translation: tx_max=+/-70, ty_max=+70, probability=50%
- random horizontal flip, probability=50%
- random brightness, probability=50%

![Generator Label Heatmnap All][image_generator_label_heatmap_all]

![Generator Label Heatmnap Red, Yellow, Green, Off][image_generator_label_heatmap_rygo]

## Datasets

All labeled traffic lights are visualized with the following colored bounding boxes.

| Class                                            | Color  |
|:-------------------------------------------------|:-------|
| All variants of Red, RedLeft, RedRight,...       | Red    |
| Yellow                                           | Yellow |
| All variants of Green, GreenLeft, GreenRight,... | Green  |
| Off, backside, side view,...                     | Grey   |

### Bosch Small Traffic Light Dataset

Source: https://hci.iwr.uni-heidelberg.de/node/6132

| Attribute                        | Description                                                                       |
|:---------------------------------|:----------------------------------------------------------------------------------|
| Number of images                 | 5093                                                                              |
| Number of labeled traffic lights | 10756                                                                             |
| Image shape                      | 1280x720x3                                                                        |
| Image format                     | 8 bit RGB, reconstructed from RIIB 12 bit (red-clear-clear-blue), size = 1280x736 |

![Bosch Label Image Example][image_bosch_labeled_image]

![Bosch Label Histogram][image_bosch_label_hist]

The following plots display a heatmap of label positions of the class red, yellow, green and off. All other label classes like RedLeft, GreenRight, etc. are filtered out because they are not relevant for our specific TL model.

![Bosch Label Heatmap All][image_bosch_label_heatmap_all]

![Bosch Label Heatmap Red, Yellow, Green, Off][image_bosch_label_heatmap_rygo]

### SDCND Capstone Dataset

Source: https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view

#### Real Data (ROS Bags)

| Attribute                        | Description |
|:---------------------------------|:------------|
| Number of images                 | 159         |
| Number of labeled traffic lights | 159         |
| Image shape                      | 1368x1096x3 |
| Image format                     | 8 bit RGB   |

![SDCND Capstone Label Image Example][image_capstone_labeled_image]

The following plots display a heatmap of label positions of the class red, yellow, green and off. The class off is currently not available in the SDCND Capstone dataset.

![SDCND Capstone Label Heatmap All][image_capstone_label_heatmap_all]

![SDCND Capstone Label Heatmap Red, Yellow, Green, Off][image_capstone_label_heatmap_rygo]

#### Udacity Simulator Data

| Attribute                        | Description |
|:---------------------------------|:------------|
| Number of images                 | 277         |
| Number of labeled traffic lights | 670         |
| Image shape                      | 800x600x3   |
| Image format                     | 8 bit RGB   |

![SDCND Capstone Simulator Label Image Example][image_capstone_sim_labeled_image]

The following plots display a heatmap of label positions of the class red, yellow, green and off. The class off is currently not available in the SDCND Capstone dataset.

![SDCND Capstone Simulator Label Heatmap All][image_capstone_sim_label_heatmap_all]

![SDCND Capstone Simulator Label Heatmap Red, Yellow, Green, Off][image_capstone_sim_label_heatmap_rygo]

### LARA Dataset

Source: http://www.lara.prd.fr/benchmarks/trafficlightsrecognition

| Attribute                        | Description |
|:---------------------------------|:------------|
| Number of images                 | 6227        |
| Number of labeled traffic lights | 9167        |
| Image shape                      | 640x480x3   |
| Image format                     | 8 bit RGB   |

![LARA Label Image Example][image_lara_labeled_image_0]
![LARA Label Image Example][image_lara_labeled_image_1]

The following plots display a heatmap of label positions of the class red, yellow, green and ambiguous.

The ambiguous labels are critical for training because they basically contains red, yellow and green traffic light, but with the following characteristics.

- Reflection distortion. The region is a reflection of an object which seems to be a traffic light
- Light shape not valid. The light of the traffic light appears circle were it is in fact a rectangle (usually due to CCD approximation or motion blur)
- Too blurry. The traffic light is 'too' blurry during its whole timeline (usually due to vehicle turning, vehicle pitch, or potholes) (for instance, frames 3568-3616)
- Too small. The traffic light is too small during its whole timeline. (for instance, frame 9 200)
- Not facing the vehicle. The traffic light is not facing the vehicle but the light is still visible. (for instance, frame 9 302)
- Lower traffic light. The small and lower traffic lights under the big one are ignored. The latter are specific to France (for instance, frame 5 260)

![LARA Label Heatmap All][image_lara_label_heatmap_all]

![LARA Label Heatmap Red, Yellow, Green, Off][image_lara_label_heatmap_rygo]
