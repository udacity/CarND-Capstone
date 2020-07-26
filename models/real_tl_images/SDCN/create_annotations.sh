#!/bin/bash

python3 create_annotations_from_sdcn1.py \
    --input_yaml real_training_data/real_data_annotations.yaml \
    --output_yaml annotations_sdcn1_real.yaml

python3 create_annotations_from_sdcn1.py \
    --input_yaml sim_training_data/sim_data_annotations.yaml \
    --outpt_yaml annotations_sdcn1_sim.yaml

python3 create_annotations_from_sdcn2.py \
    --input_json_dir new_samples \
    --output_yaml annotations_sdcn2_sim.yaml

cat annotations_sdcn1_real.yaml \
    annotations_sdcn1_sim.yaml \
    annotations_sdcn2_real.yaml > annotations_sdcn.yaml

# if [ -e "real_data_annotations_3.yaml" ]; then
#     rm "real_data_annotations_3.yaml"
# fi

# python3 create_annotations_from_dtld.py \
#     --label_file DTLD/Bremen_all.yml \
#     --calib_dir ../dtld_parsing/calibration \
#     --data_base_dir DTLD

# python3 create_annotations_from_dtld.py \
#     --label_file DTLD/Bochum_all.yml \
#     --calib_dir ../dtld_parsing/calibration \
#     --data_base_dir DTLD

# cat real_data_annotations_1.yaml \
#     real_data_annotations_2.yaml \
#     real_data_annotations_3.yaml > real_data_annotations.yaml