#!/bin/bash

python3 create_annotations_from_real_training_data.py \
    --input_yaml real_training_data/real_data_annotations.yaml

python3 create_annotations_from_labelme.py

if [ -e "real_data_annotations_3.yaml" ]; then
    rm "real_data_annotations_3.yaml"
fi

python3 create_annotations_from_dtld.py \
    --label_file DTLD/Bremen_all.yml \
    --calib_dir ../dtld_parsing/calibration \
    --data_base_dir DTLD

python3 create_annotations_from_dtld.py \
    --label_file DTLD/Bochum_all.yml \
    --calib_dir ../dtld_parsing/calibration \
    --data_base_dir DTLD

cat real_data_annotations_1.yaml \
    real_data_annotations_2.yaml \
    real_data_annotations_3.yaml > real_data_annotations.yaml