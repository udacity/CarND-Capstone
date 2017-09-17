#! /bin/bash
set -e

# Settings from environment
UDACITY_SOURCE=${UDACITY_SOURCE:-`pwd`}
UDACITY_IMAGE=${UDACITY_IMAGE:-bydavy/carnd-capstone}

docker run --rm -it -p 4567:4567 -v "${UDACITY_SOURCE}:/udacity" ${UDACITY_IMAGE} $@
