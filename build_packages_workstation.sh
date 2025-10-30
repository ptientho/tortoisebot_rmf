#!/usr/bin/env bash

set -eo pipefail
IFS=$'\n\t'
PROJECT_NAME=tortoisebot_rmf

# Install ros2 humble desktop
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble not found. Installing..."
    cd $HOME/ros_ws/src/$PROJECT_NAME
    chmod +x install_ros_humble.sh
    ./install_ros_humble.sh
else
    echo "ROS 2 Humble is already installed."
fi


# Install deps
cd $HOME/ros_ws/src/$PROJECT_NAME/docker
sudo apt update && sudo apt upgrade -y --fix-missing
sudo apt-get install -y --no-install-recommends $(cat requirements.txt)
sudo rm -rf /var/lib/apt/lists/*

# Upgrade and update
sudo apt update && sudo apt upgrade -y
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "Default sources list already exists. Skipping rosdep init."
fi
rosdep update

# Check if Gazebo is installed
if ! command -v gazebo &> /dev/null; then
    echo "Gazebo not found. Installing Gazebo Classic..."
    cd $HOME/ros_ws/src/$PROJECT_NAME
    chmod +x install_gazebo_from_source.sh
    ./install_gazebo_from_source.sh
else
    echo "Gazebo is already installed."
fi

# Install Gazebo Fortress

# Build tortoisebot package
if [ ! -d "$HOME/ros_dep_ws" ]; then
    echo "Directory ros_dep_ws not found in $HOME. Creating it..."
    cd $HOME/ros_ws/src/$PROJECT_NAME
    chmod +x install_tortoisebot.sh
    export REMOTE_PC="true"
    ./install_tortoisebot.sh
else
    echo "Directory ros_dep_ws already exists in $HOME. Skipping TortoiseBot installation."
fi