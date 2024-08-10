#!/bin/bash

# Check if the container name is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <container_name>"
  exit 1
fi

CONTAINER_NAME=$1
IMAGE_NAME=ros_noetic_image

echo "Checking if container $CONTAINER_NAME exists..."

if [ "$(sudo docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME already exists. Doing nothing."
else
    echo "Building image $IMAGE_NAME and creating container $CONTAINER_NAME..."
    sudo docker build -t $IMAGE_NAME .
    sudo xhost +local:root
    sudo docker run -d --name $CONTAINER_NAME \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /root/ros_workspace \
        -it $IMAGE_NAME /bin/bash
    if [ $? -ne 0 ]; then
        echo "Failed to create container $CONTAINER_NAME. Check the Docker logs for more information."
        sudo xhost -local:root
        exit 1
    fi
    sudo xhost -local:root
fi