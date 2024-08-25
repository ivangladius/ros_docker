#!/bin/bash

# Check if the container name and image name are provided
if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 <container_name> <image_name>"
  exit 1
fi

CONTAINER_NAME=$1
IMAGE_NAME=$2

echo "Checking if container $CONTAINER_NAME exists..."

if [ "$(sudo docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME already exists. Doing nothing."
else
    echo "Creating container $CONTAINER_NAME from image $IMAGE_NAME..."
    sudo xhost +local:root
    sudo docker run -d --name $CONTAINER_NAME \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -it $IMAGE_NAME /bin/bash
    if [ $? -ne 0 ]; then
        echo "Failed to create container $CONTAINER_NAME. Check the Docker logs for more information."
        sudo xhost -local:root
        exit 1
    fi
    sudo xhost -local:root
fi