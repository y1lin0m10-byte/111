# Real D405 Experiment

## Principle

Do not move the KUKA arm until camera calibration, perception output and
emergency stop behavior are verified.

The first real experiment is a stationary D405 dataset capture:

```text
real D405
→ real RGB-D stream
→ synchronized dataset capture
→ wood-block perception
→ offline inspection and annotation
```

## Hardware Requirements

```text
Intel RealSense D405
USB 3 connection
KUKA KR 6 R900-2 powered but stationary
wood blocks placed on a stable table
adequate diffuse lighting
```

## Install Camera Driver

```bash
sudo apt install -y ros-humble-realsense2-camera
```

Verify the camera:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

Topics should include:

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/color/points
```

## Capture Real Data

```bash
source /opt/ros/humble/setup.bash
cd ~/designgrounded-ros2-v050
source install/setup.bash

ros2 launch designgrounded_ros2 real_d405_capture.launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  session_id:=real_session_001 \
  output_dir:=$HOME/data/real_session_001 \
  start_camera:=true
```

If the camera is already running:

```bash
ros2 launch designgrounded_ros2 real_d405_capture.launch.py \
  start_camera:=false \
  session_id:=real_session_001 \
  output_dir:=$HOME/data/real_session_001
```

Captured files:

```text
~/data/real_session_001/color/*.png
~/data/real_session_001/depth/*.png
~/data/real_session_001/metadata.jsonl
```

## Verify Perception

```bash
ros2 topic echo /detected_wood_blocks_info
ros2 topic echo /dataset_capture/status
```

The output frame is camera coordinates until hand-eye calibration is installed:

```text
camera_color_optical_frame
```

After calibration:

```bash
ros2 launch designgrounded_ros2 real_d405_capture.launch.py \
  start_camera:=false \
  target_frame:=world
```

requires a valid TF chain from the camera frame to `world`.

## Real Experiment Phases

```text
Phase 1: stationary capture, no arm motion
Phase 2: multiple manual camera poses, no arm motion
Phase 3: camera mounted on stationary wrist, manual arm poses at low speed
Phase 4: autonomous observation pose selection
Phase 5: pick-and-place after perception and safety validation
```

## Required Ground Truth

For real visual experiments, use one of:

```text
AprilTag markers attached to wood blocks
calibrated turntable and fixed placement slots
manual 3D annotation with measured reference points
```

Do not use Gazebo model states for real-data evaluation.

