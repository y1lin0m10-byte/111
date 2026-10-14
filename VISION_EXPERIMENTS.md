# Vision Experiment Protocol

## Scope

The experiment treats visual state estimation as the paper's primary
contribution. The robot arm is only used to demonstrate downstream use.

## ROS 2 Data Flow

```text
D405 color + depth + CameraInfo
→ wood_block_perception
→ /detected_wood_blocks_info
→ vision_benchmark
← /gazebo/model_states ground truth
→ /vision/metrics
```

## Scenarios

```text
clear      eight wood blocks, no clutter
random     randomized block poses plus one distractor
occluded   randomized block poses plus three occluders
clutter    randomized block poses plus five distractors
```

## Commands

```bash
source /opt/ros/humble/setup.bash
cd ~/designgrounded-ros2-v030
source install/setup.bash

ros2 launch designgrounded_ros2 vision_experiment.launch.py \
  scenario:=clear \
  random_seed:=7 \
  output_csv:=/tmp/vision_clear.csv
```

Repeat with:

```bash
scenario:=random
scenario:=occluded
scenario:=clutter
```

## Metrics

```text
precision
recall
detected count
true positive
false positive
false negative
mean matched position error
```

Live metrics:

```bash
ros2 topic echo /vision/metrics
```

Raw detections:

```bash
ros2 topic echo /detected_wood_blocks_info
```

Ground truth:

```bash
ros2 topic echo /gazebo/model_states
```

## Current Baseline

The checked-in perception node uses:

```text
HSV wood-color segmentation
depth median for each connected component
camera projection
TF2 transform to world
```

It is a reproducibility baseline, not the final paper method.

## Planned Vision Upgrades

```text
1. Replace HSV contours with RGB-D instance segmentation.
2. Add point-cloud clustering and isolated block recovery.
3. Estimate oriented 3D boxes and 6D poses.
4. Calibrate detection confidence and pose covariance.
5. Add active viewpoint selection.
6. Compare against YOLO-seg, SAM2 and ICP baselines.
```

