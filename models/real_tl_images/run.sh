#!/bin/bash

# pushd "SDCN"
# ./create_annotations.sh
# popd


python3 extract_traffic_light_sdcn.py \
  --input_yaml SDCN/real_data_annotations.yaml

tar -zcvf images.tgz images && rm -rf images