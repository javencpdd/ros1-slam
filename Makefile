SHELL := /bin/bash
ROOT := $(CURDIR)
CONSOLE := $(ROOT)/console
VERSION := $(shell cat $(ROOT)/VERSION)
PYTHON ?= python3

.PHONY: test release clean

test:
	$(PYTHON) -B $(CONSOLE)/test_voxel_stream.py
	$(PYTHON) -B $(CONSOLE)/test_agent_contract.py
	$(PYTHON) -B $(CONSOLE)/test_bt_config.py
	node $(CONSOLE)/test_mission_events.js
	$(PYTHON) -B $(CONSOLE)/ops/test_world_registry.py
	$(PYTHON) -B $(CONSOLE)/ops/test_world_scenes.py

release:
	mkdir -p $(ROOT)/dist
	tar --exclude='__pycache__' -C $(ROOT) -czf $(ROOT)/dist/ros1-slam-console-$(VERSION).tar.gz console
	$(PYTHON) -B $(ROOT)/scripts/release_manifest.py $(ROOT)/dist/ros1-slam-console-$(VERSION).tar.gz

clean:
	rm -rf $(ROOT)/dist
