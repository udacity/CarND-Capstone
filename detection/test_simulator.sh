#!/bin/bash

python src/test_model.py --model out/sim_model_export/frozen_inference_graph.pb --labels models/labelmap.pbtxt --image_path test/sim --output_path out/test/sim --classes 14
