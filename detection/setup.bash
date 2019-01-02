#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export PYTHONPATH=$PYTHONPATH:${DIR}/deps/tensorflow-models/research:${DIR}/deps/tensorflow-models/research/slim

