#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'

# Upgrade and update
sudo apt update && sudo apt upgrade -y
rosdep update

# Install deps
mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws/src
git clone https://github.com/ptientho/tortoisebot_rmf.git

cd $HOME/ros_ws/src/tortoisebot/docker
sudo apt-get install -y --no-install-recommends $(cat requirements.txt)
sudo rm -rf /var/lib/apt/lists/*

# Install ros2 humble
cd $HOME/ros_ws/src/tortoisebot
chmod +x install_ros_humble.sh
./install_ros_humble.sh

# Install gazebo Classic
cd $HOME/ros_ws/src/tortoisebot
chmod +x install_gazebo_from_source.sh
./install_gazebo_from_source.sh

# Install Gazebo Fortress

# Build tortoisebot package
cd $HOME/ros_ws/src/tortoisebot
chmod +x install_tortoisebot.sh
./install_tortoisebot.sh