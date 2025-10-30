#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'
PROJECT_NAME=tortoisebot_rmf

# Install ros2 humble bare bones
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_ros_humble_barebone.sh
./install_ros_humble_barebone.sh

# Install deps
cd $HOME/ros_ws/src/$PROJECT_NAME/docker
sudo apt update && sudo apt upgrade -y
sudo apt-get install -y --no-install-recommends $(cat requirements_robot.txt)
sudo rm -rf /var/lib/apt/lists/*

# Upgrade and update
sudo apt update && sudo apt upgrade -y
sudo rosdep init
rosdep update

# Build tortoisebot package
cd $HOME/ros_ws/src/$PROJECT_NAME
chmod +x install_tortoisebot.sh
./install_tortoisebot.sh