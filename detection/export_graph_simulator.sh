#!/bin/bash

echo "Exporting graph"

MODEL_DIR=out/model
OUTPUT_DIR=out/sim_model_export
PIPELINE_CONFIG_PATH=models/faster_rcnn_resnet101_coco_simulator.config
LIST=`ls ${MODEL_DIR} | grep model.ckpt- | grep data | grep -o -E '[0-9]+' | tr '\n' ' '`
CHECKPOINT_NUMBER=`echo ${LIST} | tr " " "\n" | sort -r -g | head -n1`
CHECKPOINT_PATH=${MODEL_DIR}/model.ckpt-${CHECKPOINT_NUMBER}

echo "Using checkpoint: ${CHECKPOINT_PATH}"

python deps/tensorflow-models/object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path ${PIPELINE_CONFIG_PATH} \
    --trained_checkpoint_prefix ${CHECKPOINT_PATH} \
    --output_directory ${OUTPUT_DIR}