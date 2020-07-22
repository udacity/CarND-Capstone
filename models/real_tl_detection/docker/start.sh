#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd -P)"

IMAGE_NAME="tl_detection"
CONTAINER_NAME="$IMAGE_NAME"

EXTRA_ARGS=(
  --name "${CONTAINER_NAME}"
  --net=host
  -v $ROOT_DIR/:/work
  -w /work
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -e "DISPLAY=$DISPLAY"
  -e "USER=$USER"
)

docker run --gpus all -id --rm \
  "${EXTRA_ARGS[@]}" \
  $IMAGE_NAME
