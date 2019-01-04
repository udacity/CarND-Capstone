#!/bin/bash
WORKDIR=$PWD

# install pre-requisites
sudo apt-get install protobuf-compiler python-pil python-lxml

# Create virtualenv
pip install virtualenv
rm -Rf venv
virtualenv --python=$(which python2) venv
source venv/bin/activate

# Get dependencies
git submodule update --init
pip install -r requirements.txt

# Make the cocoapi and install
cd deps/cocoapi/PythonAPI && make && cp -r pycocotools ${WORKDIR}/deps/tensorflow-models/
cd ${WORKDIR}

# Compile protobuf definitions
cd deps/tensorflow-models/ && protoc object_detection/protos/*.proto --python_out=.
cd ${WORKDIR}
