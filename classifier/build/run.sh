#!/bin/bash

export PYTHONPATH=$PYTHONPATH:`pwd`"../src/"
python train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/pipeline.config
python export_inference_graph.py --input_type image_tensor --pipeline_config_path training/pipeline.config --trained_checkpoint_prefix training/model.ckpt-20000 --output_directory output
python test.py output

