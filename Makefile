SHELL := /bin/bash
ROOT := $(CURDIR)
WS := $(ROOT)/ws
VERSION := $(shell cat $(ROOT)/VERSION)
NOETIC ?= /opt/ros/noetic/setup.bash
P1_DEVEL ?=
JOBS ?= 4
ROS_SETUP := source $(NOETIC) && { [ -z "$(P1_DEVEL)" ] || source "$(P1_DEVEL)"; }

.PHONY: build test install release clean

build:
	@test -f $(NOETIC) || (echo "Noetic not found: source ROS1 first (or run inside the sim container)"; exit 2)
	$(ROS_SETUP) && cd $(WS) && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(JOBS)

test:
	python3 -B -m unittest discover -s $(WS)/src/v8_simulation/test -p 'test_*.py' -v
	python3 -B -m unittest discover -s $(WS)/src/lite3_gazebo_sim/test -p 'test_*.py' -v

install:
	$(ROS_SETUP) && cd $(WS) && catkin_make install -DCMAKE_BUILD_TYPE=Release -j$(JOBS)

release: install
	mkdir -p $(ROOT)/dist
	tar -C $(WS) -czf $(ROOT)/dist/ros1-slam-simulation-$(VERSION).tar.gz install
	python3 -B $(ROOT)/scripts/release_manifest.py $(ROOT)/dist/ros1-slam-simulation-$(VERSION).tar.gz

clean:
	rm -rf $(WS)/build $(WS)/devel $(WS)/install $(ROOT)/dist
