# Omron AMR package

To view the original Readme.md [click here](./docs/old/original_readme.md)

This package is a restructuring of [OmronAPAC/Omron_AMR_ROS2](https://github.com/OmronAPAC/Omron_AMR_ROS2) with a libaria-backed `amr_core` hardware interface.

View [New Developer's Guide](./docs/old/DeveloperGuide_updated.adoc).
View [Old Developer's Guide](./docs/old/DeveloperGuide.adoc).

More information
- [amr_core parameters](./docs/parameters.md)
- [Digital twin override pattern](./docs/digital-twin.md)


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
git clone --recursive https://github.com/CollaborativeRoboticsLab/omron_amr.git
```

Build by

```sh
cd ..
colcon build
```

## Usage

### Initialization

1. [Establish Remote connection to AMR Robot](./docs/old/DeveloperGuide.adoc)

2. [Configure AMR robot via Mobile Planner](./docs/old/DeveloperGuide_updated.adoc)

### Start only the hardware interface

```sh
source install/setup.bash
ros2 launch amr_ros amr_core.launch.py
```

### Start the LD250 wrapper

This launches the LD250 hardware parameters and can optionally include Nav2 and RViz.

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py
```

Example with Nav2 and RViz:

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_slam:=true rviz:=true
```

Digital twin overlay example:

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py extra_params_file:=/absolute/path/to/override.yaml
```

### Start visualization only

Run the following command to visualize the robot. Swap `ld250` with `ld90` or `amr_platform` for other robot models.

```sh
source install/setup.bash
ros2 launch amr_ros ld250_rviz.launch.py
```

### Start Teleoperation

```sh
source install/setup.bash
ros2 launch amr_teleop amr_joyop.launch.py
```

