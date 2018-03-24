# Traffic Light Classification Models
This is the documentation for the deep learning models for the traffic light detection

### Overview
We are making use of the [Tensorflow Object Detection API](https://github.com/tensorflow/models) and have chosen the following pre-trained model:

[faster_rcnn_inception_v2_coco_2018_01_28](http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz)

We have then used transfer learning to learn the new objects - specifically, traffic light colours.

### Simulator model -> tld_simulator_model

The current directory contains:

- frozen models:
  - frozen_inference_graph.pb (higher accuracy - slightly longer compute for inference)
  - faster_frozen_inference_graph.pb (slightly lower accuracy - decreased compute for inference)

- pipeline configurations 
  - pipeline.config (model trained: frozen_inference_graph.pb)
  - faster_pipeline.config (model trained: faster_frozen_inference_graph)

Note: Differences between faster_pipeline and pipeline configuration:

*faster_pipeline:* 

```
first_stage_max_proposals: 10
second_stage_batch_size: 10
max_detections_per_class: 3
max_total_detections: 6
```


*pipeline:* 

```
first_stage_max_proposals: 100
max_detections_per_class: 100
max_total_detections: 100
```

- label map: tld_simulator_label_map.pbtxt (used for both models) 
- jupyter notebook with examples of usage: tld_simulator_object_detection.ipynb

The `tld_test_images` directory contains:

- 3 sample images for testing the model via the jupyter notebook

The full data set of annotated simulator images (train/test split) can be downloaded [here](https://drive.google.com/open?id=146sr5zUg1ojYFWN0SN7_TJ41g7Jy7I9c)

### Parking Lot model (Real World) -> tld_parking_lot_model

The current directory contains:

- frozen models:
  - `frozen_inference_graph.pb` (higher accuracy - slightly longer compute for inference)
  - `faster_frozen_inference_graph.pb` (slightly lower accuracy - decreased compute for inference)

- pipeline configurations 
  - `pipeline.config` (model trained: `frozen_inference_graph.pb`)
  - `faster_pipeline.config` (model trained: `faster_frozen_inference_graph`)

Note: Differences between faster_pipeline and pipeline configuration:

*faster_pipeline:* 

```
first_stage_max_proposals: 10
second_stage_batch_size: 10
max_detections_per_class: 3
max_total_detections: 6
```


*pipeline:* 

```
first_stage_max_proposals: 100
max_detections_per_class: 100
max_total_detections: 100
```

- label map: `tld_parking_lot_label_map.pbtxt` and `faster_tld_parking_lot_label_map.pbtxt`
- jupyter notebook with examples of usage: `tld_parking_lot_object_detection.ipynb`

The `tld_test_images` directory contains:

- 3 sample images for testing the model via the jupyter notebook
