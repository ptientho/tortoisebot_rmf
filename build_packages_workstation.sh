#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'
PROJECT_NAME=tortoisebot_rmf

# Install ros2 humble desktop
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_ros_humble.sh
./install_ros_humble.sh

# Install deps
cd $HOME/ros_ws/src/tortoisebot_rmf/docker
sudo apt update && sudo apt upgrade -y
sudo apt-get install -y --no-install-recommends $(cat requirements.txt)
sudo rm -rf /var/lib/apt/lists/*

# Upgrade and update
sudo apt update && sudo apt upgrade -y
rosdep update

# Install gazebo Classic
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_gazebo_from_source.sh
./install_gazebo_from_source.sh

# Install Gazebo Fortress

# Build tortoisebot package
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_tortoisebot.sh
./install_tortoisebot.sh