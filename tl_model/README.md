# Traffic Light Detection and Classification Model

[//]: # (Image References)
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

### Required Directory Layout

- tl_model
 - datasets
    - dataset_bosch_small_tlr
      - dataset_test_rgb
      - dataset_train_rgb
    - dataset_lara
      - Lara3D_UrbanSeq1_JPG
    - dataset_sdcnd_capstone
      - real_training_data
      - sim_training_data

## DatasetHandler
To get a first impression about the dataset, run the `DatasetHandler.py` with the following arguments. It plays a short video with all labeled traffic lights for the Bosch Small Traffic Light, the LARA and the SDCND Capstone Dataset.

```
python DatasetHandler.py
  --bosch_label_file datasets/dataset_bosch_small_tlr/dataset_train_rgb/train.yaml
  --lara_label_file datasets/dataset_lara/Lara_UrbanSeq1_GroundTruth_GT.txt
  --capstone_label_file datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml
  --show_images
  --disable_filter
```

## Datasets

### Bosch Small Traffic Light Dataset

Source: https://hci.iwr.uni-heidelberg.de/node/6132

| Attribute                        | Description                                                                       |
|:---------------------------------|:----------------------------------------------------------------------------------|
| Number of images                 | 5093                                                                              |
| Number of labeled traffic lights | 10756                                                                             |
| Image shape                      | 1280x720x3                                                                        |
| Image format                     | 8 bit RGB, reconstructed from RIIB 12 bit (red-clear-clear-blue), size = 1280x736 |

All labeled traffic lights are visualized with the following colored bounding boxes.

| Class                                            | Color  |
|:-------------------------------------------------|:-------|
| All variants of Red, RedLeft, RedRight,...       | Red    |
| Yellow                                           | Yellow |
| All variants of Green, GreenLeft, GreenRight,... | Green  |
| Off                                              | Grey   |

![Bosch Label Image Example][image_bosch_labeled_image]

![Bosch Label Histogram][image_bosch_label_hist]

The following plots display a heatmap of label positions of the class red, yellow, green and off. All other label classes like RedLeft, GreenRight, etc. are filtered out because they are not relevant for our specific TL model.

![Bosch Label Heatmap All][image_bosch_label_heatmap_all]

![Bosch Label Heatmnap Red, Yellow, Green, Off][image_bosch_label_heatmap_rygo]

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

![SDCND Capstone Label Heatmnap Red, Yellow, Green, Off][image_capstone_label_heatmap_rygo]

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

![SDCND Capstone Simulator Label Heatmnap Red, Yellow, Green, Off][image_capstone_sim_label_heatmap_rygo]

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
