#!/usr/bin/env bash

echo "Training for simulator"

PIPELINE_CONFIG_PATH=models/faster_rcnn_resnet101_coco_simulator.config
MODEL_DIR=out/model

python deps/tensorflow-models/object_detection/train.py \
    --logtostderr \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --train_dir=${MODEL_DIR}
