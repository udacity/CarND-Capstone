DOCKER_NAME=udi_docker
srcdir ?= $(shell pwd)

build:
	docker build -f Dockerfile -t $(DOCKER_NAME) .

run:
	docker run -it -p 4567:4567 -v $(srcdir):/capstone $(DOCKER_NAME)

