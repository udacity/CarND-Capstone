#!/bin/bash

source /data/PythonEnv/py35-tf-latest/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/slim

python /home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/object_detection/train.py \
                    --logtostderr \
                    --pipeline_config_path=ssd_mobilenetv1_coco_pipeline.config \
                    --train_dir=run_SSD_Mobilenet

