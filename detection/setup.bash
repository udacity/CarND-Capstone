#!/bin/bash

source venv/bin/activate

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export PYTHONPATH=$PYTHONPATH:${DIR}/deps/tensorflow-models:${DIR}/deps/tensorflow-models/slim

