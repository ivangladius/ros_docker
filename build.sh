#!/bin/bash

# Check if the container name and image name are provided
if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 <container_name> <image_name>"
  exit 1
fi

CONTAINER_NAME=$1
IMAGE_NAME=$2

# Ensure the .ssh directory exists in the build context
if [ ! -d .ssh ]; then
  mkdir .ssh
fi

# Copy SSH keys to the build context
cp -r ~/.ssh/id_rsa ~/.ssh/id_rsa.pub ~/.ssh/known_hosts .ssh/

echo "Building image $IMAGE_NAME..."
sudo docker build -t $IMAGE_NAME .