# tortoisebot_rmf

## Dependencies
- ROS 2: Humble Hawksbill
- OS: Ubuntu 22.04 Jammy Jellyfish
- Simulation & Real robot
- Task planning

## Installation
```
mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws/src
git clone https://github.com/ptientho/tortoisebot_rmf.git
```

## Build
```
cd $HOME/ros_ws/src/tortoisebot_rmf
chmod +x build_packages_for_robot.sh build_packages_workstation.sh
```

## Usage

### Simulation
1. Launch simulation
```
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```

### Real robot

1. Bringup robot