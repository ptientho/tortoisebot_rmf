# tortoisebot_rmf

## Dependencies
- ROS 2: Humble Hawksbill
- OS: Ubuntu 22.04 Jammy Jellyfish
- Simulation & Real robot
- Task planning

## Installation

## Build

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