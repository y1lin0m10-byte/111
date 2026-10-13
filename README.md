# ROS 2 and Gazebo Simulation

This workspace simulates:

```text
KUKA KR 6 R900-2
Intel RealSense D405 wrist camera
small modular wood structure with 52 components
sequential pick-and-place style assembly demonstration
```

The URDF is a kinematic approximation for ROS 2 and Gazebo. It is not a
safety-certified KUKA controller model.

## Target Environment

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Classic 11
```

## Install

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-controller-manager
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch designgrounded_ros2 sim_gazebo.launch.py
```

Or use the quick-start script:

```bash
bash quick_start.sh
```

Useful topics:

```text
/d405/color/image_raw
/d405/depth/image_raw
/d405/depth/points
/assembly_progress
/joint_states
```

The assembly node moves the six joints through a demonstration sequence and
spawns components one at a time. Component catalog and poses are generated in
`designgrounded_ros2/component_catalog.py`.

## Validation without ROS 2

```bash
python3 scripts/validate_ros2_workspace.py
```

This checks XML, YAML, Python syntax, 52 unique components, D405 metadata and
workspace reach.

