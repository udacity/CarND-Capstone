#!/bin/bash
WORKDIR=$PWD

# Create virtualenv
rm -Rf venv
virtualenv --python=$(which python2) venv
source venv/bin/activate

# Get dependencies
git submodule update --init
pip install -r requirements.txt

# Make the cocoapi and install
cd deps/cocoapi/PythonAPI && make && cp -r pycocotools ${WORKDIR}/deps/tensorflow-models/research/
cd ${WORKDIR}

# Compile protobuf definitions
cd deps/tensorflow-models/research/ && protoc object_detection/protos/*.proto --python_out=.
cd ${WORKDIR}
