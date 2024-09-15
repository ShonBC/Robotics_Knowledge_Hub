# Use an official Ubuntu image as a parent image
FROM ubuntu:20.04

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    cmake \
    g++ \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir -r python/requirements.txt

# Build C++ project
RUN mkdir -p /app/c++/build && \
    cd /app/c++/build && \
    cmake .. && \
    make

# Set environment variable for library path
ENV LD_LIBRARY_PATH=/app/path_planning/c++/build:$LD_LIBRARY_PATH

# Create a script to run both Python and C++ implementations
RUN echo '#!/bin/bash\n\
python3 /app/path_planning/python/planner.py\n\
/app/path_planning/c++/build/bin/path_planner' > /app/run_planners.sh && \
    chmod +x /app/run_planners.sh

# Run the script when the container launches
CMD ["/app/run_planners.sh"]
