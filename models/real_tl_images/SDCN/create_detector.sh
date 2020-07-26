#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

# Install object_detection_api
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/models/research
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/models/research/slim
python3 ${ROOT_DIR}/models/research/object_detection/builders/model_builder_test.py

# Download pretrained model
MODEL_NAME="ssd_inception_v2_coco_2017_11_17"
if [ ! -e ${ROOT_DIR}/pretrained_models/${MODEL_NAME} ]; then
    wget http://download.tensorflow.org/models/object_detection/${MODEL_NAME}.tar.gz \
    && mkdir -p ${ROOT_DIR}/pretrained_models/ \
    && tar -zxf ${ROOT_DIR}/${MODEL_NAME}.tar.gz \
    && rm ${ROOT_DIR}/${MODEL_NAME}.tar.gz \
    && mv ${ROOT_DIR}/${MODEL_NAME} ${ROOT_DIR}/pretrained_models/
fi

pushd ${ROOT_DIR}/models/research
protoc object_detection/protos/*.proto --python_out=.
popd

# Start training
python3 ${ROOT_DIR}/models/research/object_detection/train.py \
    --logtostderr \
    --train_dir=${ROOT_DIR}/detector_training \
    --pipeline_config_path=${ROOT_DIR}/pipeline_config/${MODEL_NAME}.config

python3 ${ROOT_DIR}/models/research/object_detection/eval.py \
    --logtostderr \
    --checkpoint_dir ${ROOT_DIR}/detector_training \
    --eval_dir=${ROOT_DIR}/detector_evaluation \
    --pipeline_config_path ${ROOT_DIR}/pipeline_config/${MODEL_NAME}.config