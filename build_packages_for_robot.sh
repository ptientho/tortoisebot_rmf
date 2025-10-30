#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'
PROJECT_NAME=tortoisebot_rmf

# Check if ROS 2 Humble is installed, if not, install it
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble not found. Installing..."
    cd $HOME/ros_ws/src/$PROJECT_NAME
    chmod +x install_ros_humble_barebone.sh
    ./install_ros_humble_barebone.sh
else
    echo "ROS 2 Humble is already installed."
fi

# Install deps
cd $HOME/ros_ws/src/$PROJECT_NAME/docker
sudo apt update && sudo apt upgrade -y --fix-missing
sudo apt-get install -y --no-install-recommends $(cat requirements_robot.txt)
sudo rm -rf /var/lib/apt/lists/*

# Upgrade and update
sudo apt update && sudo apt upgrade -y
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "Default sources list already exists. Skipping rosdep init."
fi
rosdep update

# Build tortoisebot package
if [ ! -d "$HOME/ros_dep_ws" ]; then
    echo "Directory ros_dep_ws not found in $HOME. Creating it..."
    cd $HOME/ros_ws/src/$PROJECT_NAME
    chmod +x install_tortoisebot.sh
    ./install_tortoisebot.sh
else
    echo "Directory ros_dep_ws already exists in $HOME. Skipping TortoiseBot installation."
fi
