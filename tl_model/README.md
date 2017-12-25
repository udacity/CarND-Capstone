# Traffic Light Detection and Classification Model

[//]: # (Image References)
[image_bosch_labeled_image]: ./images/bosch_labeled_image_0.png
[image_bosch_label_hist]: ./images/bosch_label_histogram.png
[image_bosch_label_heatmap_all]: ./images/bosch_label_heatmap_all.png
[image_bosch_label_heatmap_rygo]: ./images/bosch_label_heatmap_red_yellow_green_off.png

[image_capstone_labeled_image]: ./images/capstone_labeled_image_0.jpg
[image_capstone_label_hist]: ./images/capstone_label_histogram.png
[image_capstone_label_heatmap_all]: ./images/capstone_label_heatmap_all.png
[image_capstone_label_heatmap_rygo]: ./images/capstone_label_heatmap_red_yellow_green_off.png

## Datasets

### Required Directory Layout

- datasets
  - dataset_bosch_small_tlr
    - dataset_test_rgb
    - dataset_train_rgb
  - dataset_lara
    - Lara3D_UrbanSeq1_JPG
  - dataset_sdcnd_capstone
    - real_training_data
    - sim_training_data

### Bosch Small Traffic Light Dataset

| Attribute                        | Description                                                                       |
|:---------------------------------|:----------------------------------------------------------------------------------|
| Number of images                 | 5093                                                                              |
| Number of labeled traffic lights | 10756                                                                             |
| Image shape                      | 1280x720x3                                                                        |
| Image format                     | 8 bit RGB, reconstructed from RIIB 12 bit (red-clear-clear-blue), size = 1280x736 |

To get a first impression about the Bosch dataset run the `DatasetHandler.py` with the following arguments. It plays a short video with all labeled traffic lights.

```
python DatasetHandler.py --bosch_label_file datasets/dataset_bosch_small_tlr/dataset_train_rgb/train.yaml --show_images
```

All labeled traffic lights are visualized with the following colored bounding boxes.

| Class                                            | Color  |
|:-------------------------------------------------|:-------|
| All variants of Red, RedLeft, RedRight,...       | Red    |
| Yellow                                           | Yellow |
| All variants of Green, GreenLeft, GreenRight,... | Green  |
| Off                                              | Grey   |

![Bosch Label Image Example][image_bosch_labeled_image]

![Bosch Label Histogram][image_bosch_label_hist]

The following images display a heatmap of label positions of the class red, yellow, green and off. All other label classes like RedLeft, GreenRight, etc. are filtered out because they are not relevant for our specific TL model.

![Bosch Label Heatmap All][image_bosch_label_heatmap_all]

![Bosch Label Heatmnap Red, Yellow, Green, Off][image_bosch_label_heatmap_rygo]

### SDCND Capstone Dataset

| Attribute                        | Description |
|:---------------------------------|:------------|
| Number of images                 | 159         |
| Number of labeled traffic lights | 159         |
| Image shape                      | 1368x1096x3 |
| Image format                     | 8 bit RGB   |

![SDCND Capstone Label Image Example][image_capstone_labeled_image]

The following images display a heatmap of label positions of the class red, yellow, green and off. The class off us currently not available in the SDCND Capstone dataset.

![SDCND Capstone Label Heatmap All][image_capstone_label_heatmap_all]

![SDCND Capstone Label Heatmnap Red, Yellow, Green, Off][image_capstone_label_heatmap_rygo]
