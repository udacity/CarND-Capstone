#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

./SDCN/create_dataset.sh

./SDCN/create_detector.sh