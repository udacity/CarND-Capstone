nvidia-docker run --rm -it \
   --user=$(id -u) \
   --env="DISPLAY" \
   --workdir=/capstone \
   --volume="$PWD":/app \
   --volume="/etc/group:/etc/group:ro" \
   --volume="/etc/passwd:/etc/passwd:ro" \
   --volume="/etc/shadow:/etc/shadow:ro" \
   --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   -p 4567:4567 -p 8888:8888 -v $PWD:/capstone -v /tmp/log:/root/.ros/
   capstone-gpu
