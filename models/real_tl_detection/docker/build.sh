#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

IMAGE_NAME="tl_detection"

docker build -t "$IMAGE_NAME" "$ROOT_DIR"
