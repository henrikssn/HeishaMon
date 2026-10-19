SHELL := /bin/bash
ARDUINO_CLI ?= $(shell command -v arduino-cli 2>/dev/null)

.PHONY: all test clean build build-binary build-esp8266 build-esp32 build-docker check-arduino-cli

all: test

test:
	$(MAKE) -C tests test

clean:
	$(MAKE) -C tests clean
	rm -f HeishaMon/*.bin HeishaMon/*.map

check-arduino-cli:
	@if [ -z "$(ARDUINO_CLI)" ]; then \
		echo "ERROR: arduino-cli is not installed or not in PATH."; \
		echo "To build without local arduino-cli, you can run: make build-docker"; \
		echo "Or install arduino-cli and required boards/libraries (see AGENTS.md)."; \
		exit 1; \
	fi

build-esp8266: check-arduino-cli
	@echo "Building HeishaMon for ESP8266 (small PCB)..."
	./scripts/build_esp8266.sh

build-esp32: check-arduino-cli
	@echo "Building HeishaMon for ESP32-S3 (large PCB)..."
	./scripts/build_esp32s3.sh

build: build-esp8266 build-esp32

build-binary: build

build-docker:
	@echo "Building HeishaMon binaries using devcontainer image..."
	docker run --rm -v "$$(pwd):/workspace" -w /workspace ghcr.io/the78mole/heishamon-dev:latest bash -c "./scripts/build_esp8266.sh && ./scripts/build_esp32s3.sh"
