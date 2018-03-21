#! /bin/bash -e

# Start (NVIDIA) docker container (named "ros") (remove after exit)
# (adapted from Udacity SDC-ND Capstone Project Starter Code)
sudo docker run --runtime=nvidia -p 4567:4567 -v $PWD:/capstone\
  -v /tmp/log:/root/.ros/ --rm --name ros -it capstone-gpu
