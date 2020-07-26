#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

# Create dataset from dataset-sdcnd-capstone (real)

# if [ ! -e ${ROOT_DIR}/dataset_sdcd1_real ]; then
#   wget -P ${ROOT_DIR} https://www.dropbox.com/s/bvq7q1zoex3b46i/dataset-sdcnd-capstone.zip
#   unzip ${ROOT_DIR}/dataset-sdcnd-capstone.zip -d ${ROOT_DIR} \
#     && mv ${ROOT_DIR}/data/real_training_data ${ROOT_DIR}/dataset_sdcd1_real \
#     && mv ${ROOT_DIR}/data/sim_training_data ${ROOT_DIR}/dataset_sdcd1_sim \
#     && rm ${ROOT_DIR}/real_data.record \
#     && rm ${ROOT_DIR}/sim_data.record \
#     && rm ${ROOT_DIR}/dataset-sdcnd-capstone.zip \
#     && rmdir ${ROOT_DIR}/data
# fi

# python3 ${ROOT_DIR}/create_annotations_from_sdcn1.py \
#     --input_yaml ${ROOT_DIR}/dataset_sdcd1_real/real_data_annotations.yaml \
#     --output_yaml ${ROOT_DIR}/annotations_sdcn1_real.yaml

# python3 ${ROOT_DIR}/create_annotations_from_sdcn1.py \
#     --input_yaml ${ROOT_DIR}/dataset_sdcd1_sim/sim_data_annotations.yaml \
#     --output_yaml ${ROOT_DIR}/annotations_sdcn1_sim.yaml

# # Create dataset from annotated dataset (real)

# if [ ! -e ${ROOT_DIR}/dataset_sdcd2_real ]; then
#   tar -zxf ${ROOT_DIR}/../data/new_samples.tgz -C ${ROOT_DIR}/ \
#     && mv ${ROOT_DIR}/new_samples ${ROOT_DIR}/dataset_sdcd2_real
# fi

# python3 ${ROOT_DIR}/create_annotations_from_sdcn2.py \
#     --input_json_dir ${ROOT_DIR}/dataset_sdcd2_real \
#     --output_yaml ${ROOT_DIR}/annotations_sdcn2_real.yaml

# # Merge annotations
# cat ${ROOT_DIR}/annotations_sdcn1_real.yaml \
#     ${ROOT_DIR}/annotations_sdcn1_sim.yaml \
#     ${ROOT_DIR}/annotations_sdcn2_real.yaml > ${ROOT_DIR}/annotations_sdcn.yaml

# # Crop images based on bounding box annotations
# python3 ${ROOT_DIR}/create_classification_dataset.py \
#   --input_yaml ${ROOT_DIR}/annotations_sdcn.yaml \
#   --output_dir ${ROOT_DIR}/dataset_classification

# Create object detection dataset

# Download tensorflow/models
if [ ! -e ${ROOT_DIR}/models ]; then
    pushd ${ROOT_DIR}
    git clone https://github.com/tensorflow/models.git
    popd

    pushd ${ROOT_DIR}/models
    git checkout 1f34fcafc1454e0d31ab4a6cc022102a54ac0f5b
    popd
    
    pushd ${ROOT_DIR}/models/research
    protoc object_detection/protos/*.proto --python_out=.
    popd
fi

# Install object_detection_api
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/models/research
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/models/research/slim
python3 ${ROOT_DIR}/models/research/object_detection/builders/model_builder_test.py

# Create dataset for object detection
DATA_DIR=${ROOT_DIR}/dataset_detection
if [ ! -e ${DATA_DIR} ]; then
    mkdir -p ${DATA_DIR}
fi
python3 ${ROOT_DIR}/create_detection_dataset.py \
    --input_yaml=${ROOT_DIR}/annotations_sdcn.yaml \
    --train_output_path=${DATA_DIR}/train_data.record \
    --valid_output_path=${DATA_DIR}/valid_data.record

# git clone https://github.com/julimueller/dtld_parsing
# cd dtld_parsing && python3 setup.py install


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