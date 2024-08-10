#!/bin/bash

# Check if the container name is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <container_name>"
  exit 1
fi

CONTAINER_NAME=$1

# Allow connections from the Docker container to the X server
echo "Allowing connections from the Docker container to the X server..."
xhost +local:root

# Ensure the DISPLAY environment variable is set correctly
export DISPLAY=:0

# Check if the container is running
if [ "$(sudo docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Connecting to the running container $CONTAINER_NAME..."
    sudo docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Container $CONTAINER_NAME is not running. Please start the container first."
    xhost -local:root
    exit 1
fi

# Optionally, remove the access control after you are done
echo "Removing access control for the X server..."
xhost -local:root