#!/bin/bash

source /data/PythonEnv/py35-tf-latest/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/:/home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/slim

# compile model
CUDA_VISIBLE_DEVICES="0" python /home/s.gerlach@de.eu.local/Projects/Repositories/ObjectDetectionAPI/research/object_detection/export_inference_graph.py \
                    --input_type image_tensor \
                    --pipeline_config_path=ssd_mobilenetv1_coco_pipeline.config \
                    --trained_checkpoint_prefix=run_SSD_Mobilenet/model.ckpt-79266 \
                    --output_directory=graphexport_SSD_3




