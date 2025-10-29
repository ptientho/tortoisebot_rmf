# Create ROS2 workspace
source /opt/ros/humble/setup.bash
mkdir -p ~/ros_dep_ws/src
cd ~/ros_dep_ws/
colcon build
source ~/ros_dep_ws/install/setup.bash

# Install dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-cartographer \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-nav2* \
    ros-$ROS_DISTRO-urdf

# Clone TortoiseBot repository
cd ~/ros_dep_ws/src
git clone -b ros2-humble https://github.com/rigbetellabs/tortoisebot.git

# Build and install ydlidar_sdk
cd ~/ros_dep_ws/src/YDLidar-SDK/build
cmake ..
make
sudo make install

# Build packages
cd ~/ros_dep_ws
rosdep install -y -i --from-paths src
colcon build

source /opt/ros/humble/setup.bash
source ~/ros_dep_ws/install/setup.bash
