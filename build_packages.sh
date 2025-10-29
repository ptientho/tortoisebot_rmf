#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'
PROJECT_NAME=tortoisebot_rmf

# Upgrade and update
sudo apt update && sudo apt upgrade -y
rosdep update

# Install ros2 humble
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_ros_humble.sh
./install_ros_humble.sh

# Install gazebo Classic
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_gazebo_from_source.sh
./install_gazebo_from_source.sh

# Install Gazebo Fortress

# Build tortoisebot package
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_tortoisebot.sh
./install_tortoisebot.sh