To reproduce training:

1. Download pretrained model http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz 
2. Download file from https://hci.iwr.uni-heidelberg.de/node/6132, this will have 4 zip files you need to unzip them with a special unarchiver find it here https://hiro.bsd.uchicago.edu/node/3168
3. Now that you have the data, you need to convert the data into TFRecords, go into the bosch_data folder where you will find the Bosch to TFRecords.ipynb, and also the bosch_label_map.pbtxt (which is needed for training)
4. Go into the notebook and set the path to the downloaded dataset, and the yaml file to create bosch.record
5. To do training, pull down models repo from Tensorflow "git pull https://github.com/tensorflow/models.git"
6. Now to train the object detection models we need to install some stuff https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md
7. To train the model: python path/to/research/models/object_detection/train.py --pipeline_config_path=faster_rcnn-traffic-bosch.config --train_dir=faster-rcnn-bosch
8. To freeze the model to be exported: python ../models/research/object_detection/export_inference_graph.py --pipeline_config_path=config/faster_rcnn-traffic-bosch.config --trained_checkpoint_prefix=faster-rcnn-bosch/model.ckpt-15000 --output_directory=faster-rcnn-traffic-bosch-frozen-sim/
9. To look at the results, go to TrafficLightDetection-Inference.ipynb and load up the model

