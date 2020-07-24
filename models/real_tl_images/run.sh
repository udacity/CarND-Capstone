#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

if [ ! -e "$ROOT_DIR/SDCN/real_training_data" ]; then
  pushd $ROOT_DIR
  wget https://www.dropbox.com/s/bvq7q1zoex3b46i/dataset-sdcnd-capstone.zip
  unzip dataset-sdcnd-capstone.zip \
    && mv data/real_training_data SDCN \
    && rm real_data.record sim_data.record \
    && rm -rf data/sim_training_data \
  popd
fi

if [ ! -e "$ROOT_DIR/SDCN/new_samples" ]; then
  tar -zxvf $ROOT_DIR/data/new_samples.tgz -C $ROOT_DIR/SDCN/
fi

pushd $ROOT_DIR/SDCN
./create_annotations.sh
popd


python3 $ROOT_DIR/extract_traffic_light_sdcn.py \
  --input_yaml SDCN/real_data_annotations.yaml

tar -zcvf $ROOT_DIR/images.tgz $ROOT_DIR/images && rm -rf $ROOT_DIR/images