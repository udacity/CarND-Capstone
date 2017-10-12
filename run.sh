#! /bin/bash
set -e

# Settings from environment
UDACITY_SOURCE=${UDACITY_SOURCE:-`pwd`}
UDACITY_IMAGE=${UDACITY_IMAGE:-capstone}
CONTAINER_NAME="udacity_carnd_capstone"

if [ "$(docker ps -a | grep ${CONTAINER_NAME})" ]; then
  echo "Attaching to running container..."
  docker exec -it ${CONTAINER_NAME} bash $@
else
  docker run --name ${CONTAINER_NAME} --rm -it -p 4567:4567 -v "${UDACITY_SOURCE}:/udacity" -v "${UDACITY_SOURCE}/log:/home/udacity/.ros/log" ${UDACITY_IMAGE} $@
fi
