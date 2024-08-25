#!/bin/bash

# Exit on any error
set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Create a catkin workspace if it doesn't exist
if [ ! -d ~/catkin_ws/src ]; then
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
fi

# Source the workspace
source ~/catkin_ws/devel/setup.bash

# Navigate to the src directory of the catkin workspace
cd ~/catkin_ws/src

# Create a new ROS package if it doesn't exist
if [ ! -d my_ros_package ]; then
    catkin_create_pkg my_ros_package rospy std_msgs
fi

# Navigate back to the catkin workspace and build
cd ~/catkin_ws
catkin_make

# Source the workspace again to ensure the new package is included
source ~/catkin_ws/devel/setup.bash

echo "Catkin workspace and package setup complete."
