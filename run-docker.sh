nvidia-docker run --rm -it \
   --env="DISPLAY" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   -p 4567:4567 -p 8888:8888 -v $PWD:/capstone -v /tmp/log:/root/.ros/ \
   --workdir=/capstone \
   capstone-gpu

xhost +local:root # for the lazy and reckless

