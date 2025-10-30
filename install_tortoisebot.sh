# Create ROS2 workspace
export ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/ros_dep_ws/src
cd ~/ros_dep_ws/
colcon build
source ~/ros_dep_ws/install/setup.bash

# Clone TortoiseBot repository
cd ~/ros_dep_ws/src
git clone -b ros2-humble https://github.com/rigbetellabs/tortoisebot.git

# Build and install ydlidar_sdk
cd ~/ros_dep_ws/src/tortoisebot/YDLidar-SDK/build
cmake ..
make
sudo make install

# Build packages
cd ~/ros_dep_ws
rosdep install -y -i --from-paths src
colcon build --parallel-workers 1

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros_dep_ws/install/setup.bash
