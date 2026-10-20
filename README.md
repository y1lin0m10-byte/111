# ROS 2 and Gazebo Simulation

This workspace simulates:

```text
KUKA KR 6 R900-2 official ROS-Industrial geometry and meshes
Intel RealSense D405 wrist camera
small modular wood structure with 52 components
sequential pick-and-place style assembly demonstration
```

There are two launch modes:

```text
sim_gazebo.launch.py   sequential component-spawning demonstration
pick_place.launch.py   D405 perception and closed-loop wood-block placement
```

The KUKA geometry, joint axes and joint limits come from
`ros-industrial/kuka_experimental`. The Gazebo D405 attachment and control
configuration are added by this project. This is not a safety-certified KUKA
controller model.

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
  ros-humble-controller-manager \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-tf2-geometry-msgs \
  python3-opencv
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

For camera-based block transfer:

```bash
ros2 launch designgrounded_ros2 pick_place.launch.py
```

Data flow:

```text
/d405/color/image_raw + /d405/depth/image_raw
→ wood_block_perception
→ /detected_wood_blocks
→ wood_pick_place
→ joint trajectory controller
→ Gazebo SetEntityState
```

Additional topics:

```text
/detected_wood_blocks
/detected_wood_blocks_info
/pick_place_progress
/vision/visual_state
/vision/uncertainty
/vision/active_observation
/vision/instance_masks
/vision/visualization
```

The classical RGB-D vision core provides:

```text
wood instance segmentation
metric point-cloud recovery
depth outlier removal
PCA-oriented bounding box
6D pose and covariance
target-slot matching
uncertainty and active-observation ranking
RViz marker visualization
```

For repeatable perception experiments:

```bash
ros2 launch designgrounded_ros2 vision_experiment.launch.py \
  scenario:=clear \
  output_csv:=/tmp/vision_clear.csv
```

The experiment scenarios and metrics are documented in
`VISION_EXPERIMENTS.md`.

For stationary real D405 capture without Gazebo:

```bash
ros2 launch designgrounded_ros2 real_d405_capture.launch.py \
  session_id:=real_session_001 \
  output_dir:=$HOME/data/real_session_001
```

See `REAL_EXPERIMENT.md` for calibration, capture and safety requirements.

The assembly node moves the six joints through a demonstration sequence and
spawns components one at a time. Component catalog and poses are generated in
`designgrounded_ros2/component_catalog.py`.

## Validation without ROS 2

```bash
python3 scripts/validate_ros2_workspace.py
```

This checks XML, YAML, Python syntax, 52 unique components, D405 metadata and
workspace reach.

