#!/bin/bash

python3 extract_traffic_light_dtld.py \
  --label_file DTLD/Bochum_all.yml \
  --calib_dir dtld_parsing/calibration \
  --data_base_dir DTLD

python3 extract_traffic_light_dtld.py \
  --label_file DTLD/Bremen_all.yml \
  --calib_dir dtld_parsing/calibration \
  --data_base_dir DTLD

python3 extract_traffic_light_sdcn.py \
  --input_yaml SDCN/real_training_data/real_data_annotations.yaml

tar -zcvf images.tgz images && rm -rf images