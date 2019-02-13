build:
	docker build . -t capstone

run:
	docker run -p 4567:4567 -v $(PWD):/capstone -v /tmp/log:/root/.ros/ --rm -it capstone

attach:
	docker exec -it $(shell sh -c "docker ps| grep capstone" | awk '{print $$1}') /bin/bash

test-init:
	wget https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/reference.bag.zip -O data/dbw_test.rosbag.bag.zip &&\
	unzip data/dbw_test.rosbag.bag.zip -d data/ &&\
	mv data/reference.bag data/dbw_test.rosbag.bag
