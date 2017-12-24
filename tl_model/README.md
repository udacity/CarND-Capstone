# Traffic Light Detection and Classification Model

[//]: # (Image References)
[image_bosch_label_hist]: ./images/bosch_label_histogram.png
[image_bosch_label_heatmap_all]: ./images/bosch_label_heatmap_all.png
[image_bosch_label_heatmap_rygo]: ./images/bosch_label_heatmap_red_yellow_green_off.png

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

![Bosch Label Histogram][image_bosch_label_hist]

The following to images display a heatmap of label positions of the class red, yellow, green and off. All other label classes like RedLeft, GreenRight, 
etc. are ignored because they are not relevant for our TL model.
  
![Bosch Label Heatmap All][image_bosch_label_heatmap_all]

![Bosch Label Heatmnap Red, Yellow, Green, Off][image_bosch_label_heatmap_rygo]