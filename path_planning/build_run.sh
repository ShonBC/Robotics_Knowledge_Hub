#!/bin/bash

# Build the Docker image for path_planning
docker build -t path_planning .

# Run the Docker container with display support for local Linux environment
# This assumes you're running Docker on a Linux system with X11

# Set the DISPLAY environment variable
export DISPLAY=host.docker.internal:0.0

# Ensure X11 is running on your Linux machine

# Run the Docker container
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/app \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    path_planning

echo "Docker container for path_planning has finished running."
