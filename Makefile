build:
	docker build . -t capstone

run:
	docker run -p 4567:4567 -v $(PWD):/capstone -v /tmp/log:/root/.ros/ --rm -it capstone

attach:
	docker exec -it $(shell sh -c "docker ps| grep capstone" | awk '{print $$1}') /bin/bash
