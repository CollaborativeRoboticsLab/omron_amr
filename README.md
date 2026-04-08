# Omron AMR package

To view the original Readme.md [click here](./docs/original_readme.md)

This package is a restructuring of [OmronAPAC/Omron_AMR_ROS2](https://github.com/OmronAPAC/Omron_AMR_ROS2)

View [New Developer's Guide](https://github.com/CollaborativeRoboticsLab/omron_amr/blob/master/docs/DeveloperGuide_updated.adoc).
View [Old Developer's Guide](https://github.com/CollaborativeRoboticsLab/omron_amr/blob/master/docs/DeveloperGuide.adoc).

| Branch | ROS2 Version | Compile |
|--------|--------------|---------|
| main | Jazzy | [![main](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Amain) |
| develop | Jazzy | [![develop](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=develop)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Adevelop) |
| humble | Humble | [![humble](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=humble)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Ahumble) |

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-teleop-twist-joy ros-humble-joy
```

Clone the repositories into the `src` folder by

```sh
git clone  --recurse-submodules https://github.com/CollaborativeRoboticsLab/omron_amr.git
```

Build by

```sh
cd ..
colcon build
```

## Usage 

### Start the robot base LD90

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py
```

to start with the rviz visualization, run

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py rviz:=true
```

### Start the robot base LD250

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py
```

to start with the rviz visualization, run

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py rviz:=true
```

### Start only the hardware interface for any robot base

```sh
source install/setup.bash
ros2 launch amr_ros amr_core.launch.py
```

### Start Teleoperation

```sh
source install/setup.bash
ros2 launch amr_teleop amr_joyop.launch.py
```

