if [ ! -d "./container_logs" ]; then
  mkdir container_logs
fi
docker run -p 4567:4567 -v $PWD:/capstone -v $PWD/container_logs:/root/.ros/ --rm -it capstone
