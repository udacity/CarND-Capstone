#!/bin/bash

source /data/PythonEnv/py35-tf-latest/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/slim

python /home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/object_detection/eval.py \
                    --logtostderr \
                    --pipeline_config_path=ssd_mobilenetv1_coco_pipeline.config \
                    --checkpoint_dir=run_SSD_Mobilenet \
                    --eval_dir=run_SSD_Mobilenet