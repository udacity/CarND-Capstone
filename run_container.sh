if [ ! -d "./container_logs" ]; then
  mkdir container_logs
fi
docker run -p 4567:4567 -p 11311:11311 -v $PWD:/capstone -v $PWD/container_logs:/root/.ros/ --rm -it --name capstone capstone
