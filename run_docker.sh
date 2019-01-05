#!/bin/bash
docker run -p 4567:4567 -v $(pwd -P):/capstone -v $(pwd -P)/logs:/root/.ros/log --rm -it capstone