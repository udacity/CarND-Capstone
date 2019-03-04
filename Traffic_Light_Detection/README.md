# Traffic Light Detection model training

To drive the car we need to be able to detect traffic lights color, for this we train an object detector using tensor flow.
The model will predict the bounding box of the traffic light, and it will have 3 classes Red, Yellow, Green.

## Preparing the training data
We will use the [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132) to train a network with the desired 3 classes, using Transfer learning on the ssd_mobilenet_coco model

To start with clone these 2 repos and download the dataset. if you place the folders like this you should be able to just run the commands as is.

    .
    ├── CarND-Capstone          # This repo
    ├── models                  # tensorflow models repo https://github.com/tensorflow/models.git
    └── dataset                 # dataset
        └── Bosch               # Bosch dataset
            ├── test            
            ├── train
            ├── test.yaml
            └── train.yaml

```console
$ cp -r models/research/object_detection CarND-Capstone/Traffic_Light_Detection/
$ python3 CarND-Capstone/Traffic_Light_Detection/data_conversion_bosch.py --output_path CarND-Capstone/Traffic_Light_Detection/training/data/test.record --path_to_yaml dataset/Bosch/tes
t.yaml --path_to_images dataset/Bosch/test
$ find dataset/Bosch/train/ -type f -print0 | xargs -0 mv -t dataset/Bosch/train
$ python3 CarND-Capstone/Traffic_Light_Detection/data_conversion_bosch.py --output_path CarND-Capstone/Traffic_Light_Detection/training/data/train.record --path_to_yaml dataset/Bosch/train.yaml --path_to_images dataset/Bosch/train
```

## Retrain model
As Carla the self driving car is installed with Tensorflow 1.3.0 we need to train the model using that version of Tensorflow, and therefore we need this older pretrained model to do the transfer learning from [ssd_mobilenet_v1_coco_11_06_2017](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz)
place the model in CarND-Capstone/Traffic_Light_Detection/models/

Before we can start training using Tensorflow object detection we need to setup our environment following the [Installation instructions]
It is important to be in the /models/research dir when running 
`export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim` as we have not copied slim to CarND-Capstone

The model can now be trained
```console
$ cd CarND-Capstone/Traffic_Light_Detection/training
$ python3 train.py --logtostderr --train_dir=models/train --pipeline_config_path=ssd_mobilenet_v1_coco.config
```

## Testing model
TBD
## Deploy model
When the training is done we want to export the inference graph using `export_inference_graph.py`
```console
$ python3 export_inference_graph.py --input_type image_tensor  --pipeline_config_path ssd_mobilenet_v1_coco.config --trained_checkpoint_prefix models/train/model.ckpt-442 --output_directory fine_tuned_model
```
and then we can copy the graph to be used by the car
```console
$ cp fine_tuned_model/frozen_inference_graph.pb ../../ros/src/tl_detector/light_classification/model/
```
## Thanks
Thanks to Daniel Stang for the nice [Object Detection API Tutorial](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e) and Anthony Sarkis for his [TFRecord script](https://github.com/swirlingsand/deeper-traffic-lights/blob/master/data_conversion_bosch.py)