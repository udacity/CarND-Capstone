#!/usr/bin/env bash

echo "Downloading model"
python src/download_model.py --output_path=out/models

# Generate TF records
echo "Generate tf records for simulator"
python src/tf_record_generation.py --output_file out/sim_training_data.record --label_map models/labelmap.pbtxt --input_file data/sim_training_data/sim_data_annotations.yaml --height 600 --width 800
echo "Generate tf records for site"
python src/tf_record_generation.py --output_file out/real_training_data.record --label_map models/labelmap.pbtxt --input_file data/real_training_data/real_data_annotations.yaml --height 1096 --width 1368

