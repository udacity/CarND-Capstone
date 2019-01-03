#!/usr/bin/env bash

echo "Training for simulator"

PIPELINE_CONFIG_PATH=models/faster_rcnn_resnet101_coco.config
MODEL_DIR=out
#NUM_TRAIN_STEPS=50000
NUM_TRAIN_STEPS=1
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python deps/tensorflow-models/research/object_detection/model_main.py \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --model_dir=${MODEL_DIR} \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --sample_1_of_n_eval_examples=${SAMPLE_1_OF_N_EVAL_EXAMPLES} \
    --alsologtostderr
